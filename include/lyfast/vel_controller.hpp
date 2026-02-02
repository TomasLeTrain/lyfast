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
};

struct VelocityControllerParams {
    KvUnits left_Kv;
    KaUnits left_Ka;
    KsUnits left_Ks;
    Divided<Voltage, LinearVelocity> left_Kp { 0 };
    Divided<Voltage, Length> left_Ki { 0 };
    Voltage left_max_output { 1_volt };

    KvUnits right_Kv;
    KaUnits right_Ka;
    KsUnits right_Ks;
    Divided<Voltage, LinearVelocity> right_Kp { 0 };
    Divided<Voltage, Length> right_Ki { 0 };
    Voltage right_max_output { 1_volt };

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

            .right_Kv = params.Kv,
            .right_Ka = params.Ka,
            .right_Ks = params.Ks,
            .right_Kp = params.Kp,
            .right_Ki = params.Ki,
            .right_max_output = params.max_output,
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
            double tbh_factor = 0.8;
            current_integral *= tbh_factor;
        }

        Voltage result {
            // kv
            target * m_params.Kv +
              // ka
              target_accel * m_params.Ka +
              // ks
              units::sgn(target) * m_params.Ks +
              // kp
              m_params.Kp * error +
              // ki
              m_params.Ki * current_integral,
        };

        if (
          // currently saturating
          units::abs(result) >= m_params.max_output &&
          // output going in direct of error
          units::sgn(error) == units::sgn(result)) {
            // clamp output and stop integral windup
            result =
              units::clamp(result, -m_params.max_output, m_params.max_output);
            // no need to update integral to current integral
        } else {
            // not saturating, update integral
            integral = current_integral;
        }

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

    Length m_track_width;
    DifferentialDrivetrain& drivetrain;

  public:
    LeftRightVoltages update(LeftRightSpeeds measurement,
                             DifferentialSpeeds target,
                             Time duration) {
        Length track_radius = m_track_width / 2.0;

        // desaturate target first
        target = desaturateDifferentialSpeeds(target,
                                              m_track_width,
                                              drivetrain.getMaxVelocity());

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
        return update(drivetrain.getDrivetrainVelocities(), target, duration);
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

    DifferentialVelocityController(VelocityControllerParams params,
                                   Length track_width,
                                   DifferentialDrivetrain& drivetrain)
        : m_params(params),
          left_controller({ .Kv = this->m_params.left_Kv,
                            .Ka = this->m_params.left_Ka,
                            .Ks = this->m_params.left_Ks,
                            .Kp = this->m_params.left_Kp,
                            .Ki = this->m_params.left_Ki,
                            .max_output = this->m_params.left_max_output }),
          right_controller({
            .Kv = this->m_params.right_Kv,
            .Ka = this->m_params.right_Ka,
            .Ks = this->m_params.right_Ks,
            .Kp = this->m_params.right_Kp,
            .Ki = this->m_params.right_Ki,
            .max_output = this->m_params.right_max_output,
          }),
          m_track_width(track_width),
          drivetrain(drivetrain) {}

    DifferentialVelocityController(
      SimpleVelocityControllerParams<LinearVelocity> params,
      Length track_width,
      DifferentialDrivetrain& drivetrain)
        : m_params(VelocityControllerParams::fromSimple(params)),
          left_controller(params),
          right_controller(params),
          m_track_width(track_width),
          drivetrain(drivetrain) {}
};

class ArcadeVelocityController {
    DifferentialVelocityController linear_controller;
    DifferentialVelocityController angular_controller;

    Length m_track_width;
    DifferentialDrivetrain& drivetrain;

  public:
    LeftRightVoltages update(LeftRightSpeeds measurement,
                             DifferentialSpeeds target,
                             Time duration) {
        Voltage linear = linear_controller.update(measurement,
                                                  target.linear_velocity,
                                                  duration);
        Voltage angular = angular_controller.update(measurement,
                                                    target.angular_velocity,
                                                    duration);

        return LeftRightVoltages { linear - angular, linear + angular };
    }

    LeftRightVoltages update(DifferentialSpeeds target, Time duration) {
        Length track_radius = m_track_width / 2.0;

        target = desaturateDifferentialSpeeds(target,
                                              m_track_width,
                                              drivetrain.getMaxVelocity());

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
                             Length track_width,
                             DifferentialDrivetrain& drivetrain)
        : linear_controller(linear_controller),
          angular_controller(angular_controller),
          m_track_width(track_width),
          drivetrain(drivetrain) {}
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
    template<typename Self>
    Self with_velocity_feedforward(this Self&& self,
                                   Controller new_velocity_feedforward) {
        Self new_self = self;
        new_self.velocity_feedforward = new_velocity_feedforward;
        return new_self;
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
    template<typename Self>
    Self with_velocity_feedback(this Self&& self,
                                Controller new_velocity_feedback) {
        Self new_self = self;
        new_self.velocity_feedback = new_velocity_feedback;
        return new_self;
    }
};

template<typename Controller>
concept hasVelocityFeedback =
  requires(Controller controller) { controller.velocity_feedback; };

} // namespace lyfast
} // namespace blazing
