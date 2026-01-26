#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedforward/feedforward.hpp"
#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "lyfast/system_identification.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

template<typename T>
struct SimpleVelocityControllerParams {
    Divided<Voltage, T> Kv;
    Divided<Voltage, Divided<T, Time>> Ka;
    Voltage Ks;
    Divided<Voltage, T> Kp { 0 };
    Divided<Voltage, Multiplied<T, Time>> Ki { 0 };
};

struct VelocityControllerParams {
    KvUnits left_Kv;
    KaUnits left_Ka;
    KsUnits left_Ks;
    Divided<Voltage, LinearVelocity> left_Kp { 0 };
    Divided<Voltage, Length> left_Ki { 0 };

    KvUnits right_Kv;
    KaUnits right_Ka;
    KsUnits right_Ks;
    Divided<Voltage, LinearVelocity> right_Kp { 0 };
    Divided<Voltage, Length> right_Ki { 0 };

    // construct both sides with equal gains
    static VelocityControllerParams
    fromSimple(SimpleVelocityControllerParams<LinearVelocity> params) {
        return { .left_Kv = params.Kv,
                 .left_Ka = params.Ka,
                 .left_Ks = params.Ks,
                 .left_Kp = params.Kp,
                 .left_Ki = params.Ki,
                 .right_Kv = params.Kv,
                 .right_Ka = params.Ka,
                 .right_Ks = params.Ks,
                 .right_Kp = params.Kp,
                 .right_Ki = params.Ki };
    }
};

template<typename T>
class SimpleVelocityController {
    SimpleVelocityControllerParams<T> m_params;

    std::optional<T> last_speed = std::nullopt;

    Multiplied<T, Time> integral = 0_in;
    Multiplied<T, Time> right_integral = 0_in;

    std::optional<T> last_error = std::nullopt;

  public:
    Voltage update(T measurement, T target, Time duration) {
        LinearAcceleration target_accel =
          (target -
           // combines measurement and last_speeds
           last_speed.value_or(T(0))) /
          duration;

        LinearVelocity error = target - measurement;

        integral += error * duration;

        if (last_error && units::sgn(error) != units::sgn(*last_error)) {
            integral = Length { 0 };
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
              m_params.Ki * integral,
        };

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

class VelocityController {
    VelocityControllerParams m_params;

    SimpleVelocityController<LinearVelocity> left_controller;
    SimpleVelocityController<LinearVelocity> right_controller;

    Length m_track_width;

    DifferentialDrivetrain& drivetrain;

  public:
    LeftRightVoltages update(LeftRightSpeeds measurement,
                             DifferentialSpeeds target,
                             Time duration) {
        LinearVelocity target_left_vel =
          target.linear_velocity -
          (target.angular_velocity / rad) * (m_track_width / 2);
        LinearVelocity target_right_vel =
          target.linear_velocity +
          (target.angular_velocity / rad) * (m_track_width / 2);

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

        // LinearVelocity target_left_vel =
        //   target.linear_velocity -
        //   (target.angular_velocity / rad) * (m_track_width / 2);
        // LinearVelocity target_right_vel =
        //   target.linear_velocity +
        //   (target.angular_velocity / rad) * (m_track_width / 2);
        //
        // auto left_voltage = left_controller.update(target_left_vel,
        // duration); auto right_voltage =
        //   right_controller.update(target_right_vel, duration);
        //
        // LeftRightVoltages result = { left_voltage, right_voltage };
        //
        // return result;
    }

    // allows using as only a linear feedforward
    Voltage update(LinearVelocity target, Time duration) {
        auto left_right_voltages =
          update(DifferentialSpeeds { target, 0_radps }, duration);
        return (left_right_voltages.right_voltage +
                left_right_voltages.left_voltage) /
               2.0;
    }

    // allows using as only an angular feedforward
    Voltage update(AngularVelocity target, Time duration) {
        auto left_right_voltages =
          update(DifferentialSpeeds { 0_inps, target }, duration);

        return (left_right_voltages.right_voltage -
                left_right_voltages.left_voltage) /
               2.0;
    }

    VelocityControllerParams getParams() {
        return m_params;
    }

    VelocityController(VelocityControllerParams params,
                       Length track_width,
                       DifferentialDrivetrain& drivetrain)
        : m_params(params),
          left_controller({
            .Kv = this->m_params.left_Kv,
            .Ka = this->m_params.left_Ka,
            .Ks = this->m_params.left_Ks,
            .Kp = this->m_params.left_Kp,
            .Ki = this->m_params.left_Ki,
          }),
          right_controller({
            .Kv = this->m_params.right_Kv,
            .Ka = this->m_params.right_Ka,
            .Ks = this->m_params.right_Ks,
            .Kp = this->m_params.right_Kp,
            .Ki = this->m_params.right_Ki,
          }),
          m_track_width(track_width),
          drivetrain(drivetrain) {}

    VelocityController(SimpleVelocityControllerParams<LinearVelocity> params,
                       Length track_width,
                       DifferentialDrivetrain& drivetrain)
        : m_params(VelocityControllerParams::fromSimple(params)),
          left_controller(params),
          right_controller(params),
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
    Self with_linear_feedforward(this Self&& self,
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
    Self with_linear_feedback(this Self&& self,
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
