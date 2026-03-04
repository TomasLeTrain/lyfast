#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedforward/feedforward.hpp"
#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <functional>
#include <ios>

namespace blazing {
namespace lyfast {

// voltage is assumed to be in the range [0,1]

// u = Ks * sgn(v) + Kv * v + Ka * a;
using KsUnits = Voltage;
using KvUnits = Divided<Voltage, LinearVelocity>;
using KaUnits = Divided<Voltage, LinearAcceleration>;

using FKsUnits = FVoltage;
using FKvUnits = Divided<FVoltage, FLinearVelocity>;
using FKaUnits = Divided<FVoltage, FLinearAcceleration>;

inline DifferentialSpeeds
desaturatePrioritizeAngularDiffSpeeds(DifferentialSpeeds target,
                                      Length track_width,
                                      LinearVelocity max_velocity) {
    Length track_radius = track_width / 2.0;

    // first determine how fast we want to go angular wise
    LinearVelocity lin_alg_target =
      (target.angular_velocity / rad) * track_radius;

    // clamp linear speed based on angular speed
    LinearVelocity max_lin_speed =
      units::abs(max_velocity) - units::abs(lin_alg_target);

    LinearVelocity new_lin_speed =
      units::clamp(target.linear_velocity, -max_lin_speed, max_lin_speed);

    return { new_lin_speed, target.angular_velocity };
}

inline DifferentialSpeeds
desaturateDifferentialSpeeds(DifferentialSpeeds target,
                             Length track_width,
                             LinearVelocity max_velocity) {
    Length track_radius = track_width / 2.0;

    LinearVelocity target_left_vel =
      target.linear_velocity - (target.angular_velocity / rad) * track_radius;
    LinearVelocity target_right_vel =
      target.linear_velocity + (target.angular_velocity / rad) * track_radius;

    std::array<LinearVelocity, 2> saturated = { target_left_vel,
                                                target_right_vel };

    auto [new_left_vel, new_right_vel] =
      blazing::desaturate(saturated, max_velocity);

    LinearVelocity new_lin_vel = (new_left_vel + new_right_vel) / 2.0;
    AngularVelocity new_ang_vel =
      rad * (new_right_vel - new_left_vel) / track_width;

    return { new_lin_vel, new_ang_vel };
}

template<typename T>
struct SimpleVelocityControllerParams {
    Divided<Voltage, T> Kv;
    Divided<Voltage, Divided<T, Time>> Ka;
    Voltage Ks;
    Divided<Voltage, T> Kp { 0 };
    Divided<Voltage, Multiplied<T, Time>> Ki { 0 };

    Voltage max_output { 1_volt };

    double tbh_factor { 0.0 };
};

struct VelocityControllerParams {
    KvUnits left_Kv;
    KaUnits left_Ka;
    KsUnits left_Ks;
    Divided<Voltage, LinearVelocity> left_Kp { 0 };
    Divided<Voltage, Length> left_Ki { 0 };
    Voltage left_max_output { 1_volt };
    double left_tbh_factor { 0.0 };

    KvUnits right_Kv;
    KaUnits right_Ka;
    KsUnits right_Ks;
    Divided<Voltage, LinearVelocity> right_Kp { 0 };
    Divided<Voltage, Length> right_Ki { 0 };
    Voltage right_max_output { 1_volt };
    double right_tbh_factor { 0.0 };

    // construct both sides with equal gains
    static VelocityControllerParams
    fromSimple(SimpleVelocityControllerParams<LinearVelocity> params) {
        return {
            .left_Kv = params.Kv,
            .left_Ka = params.Ka,
            .left_Ks = params.Ks,
            .left_Kp = params.Kp,
            .left_Ki = params.Ki,
            .left_max_output = params.max_output,
            .left_tbh_factor = params.tbh_factor,

            .right_Kv = params.Kv,
            .right_Ka = params.Ka,
            .right_Ks = params.Ks,
            .right_Kp = params.Kp,
            .right_Ki = params.Ki,
            .right_max_output = params.max_output,
            .right_tbh_factor = params.tbh_factor,
        };
    }
};

template<typename T>
class SimpleVelocityController {
    SimpleVelocityControllerParams<T> m_params;

    std::optional<T> last_speed = std::nullopt;

    Multiplied<T, Time> integral = 0_in;

    std::optional<T> last_error = std::nullopt;

  public:
    Voltage update(T measurement, T target, Time duration) {
        LinearAcceleration target_accel =
          (target -
           // combines measurement and last_speeds
           last_speed.value_or(T(0))) /
          duration;

        LinearVelocity error = target - measurement;

        Multiplied<T, Time> current_integral = integral;

        if (last_error)
            // use trapezoidal approximation
            current_integral += (error + *last_error) * duration / 2.0;
        else
            // use Riemann sum approximation
            current_integral += error * duration;

        // decrease integral by some amount when crossing error to minimize
        // overshooot due to the integral
        if (last_error && units::sgn(error) != units::sgn(*last_error)) {
            current_integral *= m_params.tbh_factor;
        }

        Voltage result {
            // kv
            target * m_params.Kv +
              // ka
              target_accel * m_params.Ka +
              // kp
              m_params.Kp * error +
              // ki
              m_params.Ki * current_integral,
        };

        // base ks off of the desired voltage
        result += units::sgn(result) * m_params.Ks;

        if (
          // currently saturating
          units::abs(result) >= m_params.max_output &&
          // output going in direct of error
          units::sgn(error) == units::sgn(result)) {
            // clamping, stop integral windup
            // no need to update integral to current integral
        } else {
            // not saturating, update integral
            integral = current_integral;
        }

        result =
          units::clamp(result, -m_params.max_output, m_params.max_output);

        last_speed = { target };
        last_error = error;

        return result;
    }

    Voltage update(T target, Time duration) {
        LinearAcceleration target_accel =
          (target -
           // combines measurement and last_speeds
           last_speed.value_or(T(0))) /
          duration;

        Voltage result { // kv
                         target * m_params.Kv +
                         // ka
                         target_accel * m_params.Ka +
                         // ks
                         units::sgn(target) * m_params.Ks
        };

        last_speed = { target };

        return result;
    }

    SimpleVelocityControllerParams<T> getParams() {
        return m_params;
    }

    SimpleVelocityController(SimpleVelocityControllerParams<T> params)
        : m_params(params) {}
};

class DifferentialVelocityController {
    VelocityControllerParams m_params;

    SimpleVelocityController<LinearVelocity> left_controller;
    SimpleVelocityController<LinearVelocity> right_controller;

    LinearVelocity m_max_velocity;
    Length m_track_width;
    double m_vel_alpha = 1.0;
    bool m_prioritize_angular = false;
    std::reference_wrapper<DifferentialDrivetrain> drivetrain;

    std::optional<LeftRightSpeeds> last_velocities = std::nullopt;

  public:
    LeftRightVoltages update(LeftRightSpeeds measurement,
                             DifferentialSpeeds target,
                             Time duration) {
        Length track_radius = m_track_width / 2.0;

        // desaturate target first
        if (m_prioritize_angular)
            target = desaturatePrioritizeAngularDiffSpeeds(target,
                                                           m_track_width,
                                                           m_max_velocity);
        else
            target = desaturateDifferentialSpeeds(target,
                                                  m_track_width,
                                                  m_max_velocity);

        LinearVelocity target_left_vel =
          target.linear_velocity -
          (target.angular_velocity / rad) * track_radius;
        LinearVelocity target_right_vel =
          target.linear_velocity +
          (target.angular_velocity / rad) * track_radius;

        auto left_voltage = left_controller.update(measurement.left_vel,
                                                   target_left_vel,
                                                   duration);
        auto right_voltage = right_controller.update(measurement.right_vel,
                                                     target_right_vel,
                                                     duration);

        LeftRightVoltages result = { left_voltage, right_voltage };

        return result;
    }

    LeftRightVoltages update(DifferentialSpeeds target, Time duration) {
        // fall back to using specified drivetrain
        LeftRightSpeeds velocities = drivetrain.get().getDrivetrainVelocities();

        // use low pass filter on the velocities
        if (last_velocities) {
            velocities.left_vel =
              m_vel_alpha * velocities.left_vel +
              (1 - m_vel_alpha) * last_velocities.value().left_vel;

            velocities.right_vel =
              m_vel_alpha * velocities.right_vel +
              (1 - m_vel_alpha) * last_velocities.value().right_vel;
        }

        last_velocities = velocities;

        return update(velocities, target, duration);
    }

    // allows using as only a linear feedforward
    // NOTE: this cannot be used in combination with another controller as this
    // controller uses feedback
    Voltage update(LinearVelocity target, Time duration) {
        auto left_right_voltages =
          update(DifferentialSpeeds { target, 0_radps }, duration);
        return (left_right_voltages.right_voltage +
                left_right_voltages.left_voltage) /
               2.0;
    }

    // allows using as only a linear feedback with only linear component
    // NOTE: this cannot be used in combination with another controller as this
    // controller uses feedback
    Voltage
    update(LeftRightSpeeds measurement, LinearVelocity target, Time duration) {
        auto left_right_voltages =
          update(measurement, DifferentialSpeeds { target, 0_radps }, duration);
        return (left_right_voltages.right_voltage +
                left_right_voltages.left_voltage) /
               2.0;
    }

    // allows using as only an angular feedforward
    // NOTE: this cannot be used in combination with another controller as this
    // controller uses feedback
    Voltage update(AngularVelocity target, Time duration) {
        auto left_right_voltages =
          update(DifferentialSpeeds { 0_inps, target }, duration);

        return (left_right_voltages.right_voltage -
                left_right_voltages.left_voltage) /
               2.0;
    }

    // allows using as vel feedback with only angular component
    // NOTE: this cannot be used in combination with another controller as this
    // controller uses feedback
    Voltage
    update(LeftRightSpeeds measurement, AngularVelocity target, Time duration) {
        auto left_right_voltages =
          update(measurement, DifferentialSpeeds { 0_inps, target }, duration);

        return (left_right_voltages.right_voltage -
                left_right_voltages.left_voltage) /
               2.0;
    }

    VelocityControllerParams getParams() {
        return m_params;
    }

    DifferentialVelocityController(
      VelocityControllerParams params,
      LinearVelocity max_velocity,
      Length track_width,
      double vel_alpha,
      bool prioritize_angular,
      std::reference_wrapper<DifferentialDrivetrain> drivetrain)
        : m_params(params),
          left_controller({
            .Kv = this->m_params.left_Kv,
            .Ka = this->m_params.left_Ka,
            .Ks = this->m_params.left_Ks,
            .Kp = this->m_params.left_Kp,
            .Ki = this->m_params.left_Ki,
            .max_output = this->m_params.left_max_output,
            .tbh_factor = this->m_params.left_tbh_factor,
          }),
          right_controller({
            .Kv = this->m_params.right_Kv,
            .Ka = this->m_params.right_Ka,
            .Ks = this->m_params.right_Ks,
            .Kp = this->m_params.right_Kp,
            .Ki = this->m_params.right_Ki,
            .max_output = this->m_params.right_max_output,
            .tbh_factor = this->m_params.right_tbh_factor,
          }),
          m_max_velocity(max_velocity),
          m_track_width(track_width),
          m_vel_alpha(vel_alpha),
          m_prioritize_angular(prioritize_angular),
          drivetrain(drivetrain) {}

    DifferentialVelocityController(
      SimpleVelocityControllerParams<LinearVelocity> params,
      LinearVelocity max_velocity,
      Length track_width,
      double vel_alpha,
      bool prioritize_angular,
      std::reference_wrapper<DifferentialDrivetrain> drivetrain)
        : m_params(VelocityControllerParams::fromSimple(params)),
          left_controller(params),
          right_controller(params),

          m_max_velocity(max_velocity),
          m_track_width(track_width),
          m_vel_alpha(vel_alpha),
          m_prioritize_angular(prioritize_angular),
          drivetrain(drivetrain) {}
};

class ArcadeVelocityController {
    DifferentialVelocityController linear_controller;
    DifferentialVelocityController angular_controller;

    LinearVelocity m_max_velocity;
    bool m_prioritize_angular = false;

    Length m_track_width;

  public:
    // TODO: rewrite to actually be good
    // LeftRightVoltages update(LeftRightSpeeds measurement,
    //                          DifferentialSpeeds target,
    //                          Time duration) {
    //     Voltage linear = linear_controller.update(measurement,
    //                                               target.linear_velocity,
    //                                               duration);
    //     Voltage angular = angular_controller.update(measurement,
    //                                                 target.angular_velocity,
    //                                                 duration);
    //
    //     return LeftRightVoltages { linear - angular, linear + angular };
    // }

    LeftRightVoltages update(DifferentialSpeeds target, Time duration) {
        Length track_radius = m_track_width / 2.0;

        // desaturate target first
        if (m_prioritize_angular) {
            target = desaturatePrioritizeAngularDiffSpeeds(target,
                                                           m_track_width,
                                                           m_max_velocity);
        } else {
            target = desaturateDifferentialSpeeds(target,
                                                  m_track_width,
                                                  m_max_velocity);
        }

        LeftRightVoltages linear = linear_controller.update(target, duration);
        LeftRightVoltages angular = angular_controller.update(target, duration);

        // factor = v / (v + w * r)
        // 1 when all linear, 0 when all angular
        // defaults to linear if both linear and angular are near 0
        float lin_factor = 1.0;

        auto den = (units::abs(target.linear_velocity) +
                    units::abs(target.angular_velocity) * track_radius / rad);

        // only calculate lin_factor if den is not 0
        if (units::abs(den).internal() > 1e-5) {
            lin_factor = units::abs(target.linear_velocity) / den;
        }

        // interpolate between controllers
        Voltage left_voltage = std::lerp(angular.left_voltage.internal(),
                                         linear.left_voltage.internal(),
                                         lin_factor) *
                               volt;

        Voltage right_voltage = std::lerp(angular.right_voltage.internal(),
                                          linear.right_voltage.internal(),
                                          lin_factor) *
                                volt;

        return LeftRightVoltages { left_voltage, right_voltage };
    }

    // allows using as only a linear feedforward
    Voltage update(LinearVelocity target, Time duration) {
        return linear_controller.update(target, duration);
    }

    // allows using as linear feedback with measurement
    Voltage
    update(LeftRightSpeeds measurement, LinearVelocity target, Time duration) {
        return linear_controller.update(measurement, target, duration);
    }

    // allows using as only an angular feedforward
    Voltage update(AngularVelocity target, Time duration) {
        return angular_controller.update(target, duration);
    }

    // allows using as angular feedback with measurement
    Voltage
    update(LeftRightSpeeds measurement, AngularVelocity target, Time duration) {
        return angular_controller.update(measurement, target, duration);
    }

    ArcadeVelocityController(DifferentialVelocityController linear_controller,
                             DifferentialVelocityController angular_controller,
                             LinearVelocity max_velocity,
                             bool prioritize_angular,
                             Length track_width)
        : linear_controller(linear_controller),
          angular_controller(angular_controller),
          m_max_velocity(max_velocity),
          m_prioritize_angular(prioritize_angular),
          m_track_width(track_width) {}
};

template<typename Controller>
    requires Feedforward<Controller, DifferentialSpeeds, LeftRightVoltages>
struct VelocityFeedforward : virtual ControllerBase {
  public:
    Controller velocity_feedforward;

    VelocityFeedforward(Controller velocity_feedforward_controller)
        : velocity_feedforward(velocity_feedforward_controller) {}

    // creates a copy of the controller with different linear feedback
    // controller
    void set_velocity_feedforward(Controller new_velocity_feedforward) {
        this->velocity_feedforward = new_velocity_feedforward;
    }
};

template<typename Controller>
concept hasVelocityFeedforward =
  requires(Controller controller) { controller.velocity_feedforward; };

template<typename Controller>
    requires Feedback<Controller, DifferentialSpeeds, LeftRightVoltages>
struct VelocityFeedback : virtual ControllerBase {
  public:
    Controller velocity_feedback;

    VelocityFeedback(Controller velocity_feedforward_controller)
        : velocity_feedback(velocity_feedforward_controller) {}

    // creates a copy of the controller with different linear feedback
    // controller
    void set_velocity_feedback(Controller new_velocity_feedback) {
        this->velocity_feedback = new_velocity_feedback;
    }
};

template<typename Controller>
concept hasVelocityFeedback =
  requires(Controller controller) { controller.velocity_feedback; };

} // namespace lyfast
} // namespace blazing
