#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedforward/feedforward.hpp"
#include "blazing/utils.hpp"
#include "lyfast/system_identification.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

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
};

struct LeftRightSpeeds {
    LinearVelocity left_vel;
    LinearVelocity right_vel;
};

class VelocityController {
    VelocityControllerParams m_params;
    Length m_track_width;

    std::optional<LeftRightSpeeds> last_speeds = std::nullopt;

    Length left_integral = 0_in;
    Length right_integral = 0_in;

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

        LinearAcceleration target_left_accel =
          (target_left_vel -
           // combines measurement and last_speeds
           last_speeds
             .transform([](auto e) {
                 return e.left_vel;
             })
             .value_or(0_inps)) /
          duration;

        LinearAcceleration target_right_accel =
          (target_right_vel -
           // combines measurement and last_speeds
           last_speeds
             .transform([](auto e) {
                 return e.right_vel;
             })
             .value_or(0_inps)) /
          duration;

        LinearVelocity left_error = target_left_vel - measurement.left_vel;
        LinearVelocity right_error = target_right_vel - measurement.right_vel;

        left_integral += left_error * duration;
        right_integral += right_error * duration;

		// TODO: add integral reset

        LeftRightVoltages result {
            target_left_vel * m_params.left_Kv +
              target_left_accel * m_params.left_Ka +
              m_params.left_Kp * left_error + m_params.left_Ki * left_integral,

            target_right_vel * m_params.right_Kv +
              target_right_accel * m_params.right_Ka +
              m_params.right_Kp * right_error +
              m_params.right_Ki * right_integral,
        };

        result.left_voltage += units::sgn(target_left_vel) * m_params.left_Ks;
        result.right_voltage +=
          units::sgn(target_right_vel) * m_params.right_Ks;

        last_speeds = { target_left_vel, target_right_vel };

        return result;
    }

    LeftRightVoltages update(DifferentialSpeeds target, Time duration) {
        LinearVelocity left_vel =
          target.linear_velocity -
          (target.angular_velocity / rad) * (m_track_width / 2);
        LinearVelocity right_vel =
          target.linear_velocity +
          (target.angular_velocity / rad) * (m_track_width / 2);

        LinearAcceleration left_accel =
          (left_vel - last_speeds
                        // if no last speeds then we assume zero acceleration
                        .transform([](auto speeds) {
                            return speeds.left_vel;
                        })
                        .value_or(left_vel)) /
          duration;

        LinearAcceleration right_accel =
          (right_vel - last_speeds
                         // if no last speeds then we assume zero acceleration
                         .transform([](auto speeds) {
                             return speeds.right_vel;
                         })
                         .value_or(right_vel)) /
          duration;

        LeftRightVoltages result {
            left_vel * m_params.left_Kv + left_accel * m_params.left_Ka,
            right_vel * m_params.right_Kv + right_accel * m_params.right_Ka
        };

        result.left_voltage += units::sgn(left_vel) * m_params.left_Ks;
        result.right_voltage += units::sgn(right_vel) * m_params.right_Ks;

        last_speeds = { left_vel, right_vel };

        return result;
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

    VelocityController(VelocityControllerParams params, Length track_width)
        : m_params(params),
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
    template<typename Self>
    Self with_linear_feedback(this Self&& self,
                              Controller new_velocity_feedforward) {
        Self new_self = self;
        new_self.velocity_feedforward = new_velocity_feedforward;
        return new_self;
    }
};

template<typename Controller>
concept hasVelocityFeedforward =
  requires(Controller controller) { controller.velocity_feedforward; };

} // namespace lyfast
} // namespace blazing
