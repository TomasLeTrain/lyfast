#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedforward/feedforward.hpp"
#include "blazing/utils.hpp"
#include "lyfast/system_identification.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

struct VelocityControllerParams {
    KvUnits left_Kv;
    KaUnits left_Ka;
    KsUnits left_Ks;

    KvUnits right_Kv;
    KaUnits right_Ka;
    KsUnits right_Ks;
};

struct LeftRightSpeeds {
    LinearVelocity left_vel;
    LinearVelocity right_vel;
};

class VelocityController {
    VelocityControllerParams m_params;

    std::optional<LeftRightSpeeds> last_speeds = std::nullopt;

  public:
    DifferentialVoltages update(DifferentialSpeeds target, Time duration) {
        // TODO: make configurable
        Length track_width = 10.5_in;

        LinearVelocity left_vel =
          target.linear_velocity -
          target.angular_velocity / rad * (track_width / 2);
        LinearVelocity right_vel =
          target.linear_velocity +
          target.angular_velocity / rad * (track_width / 2);

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

        DifferentialVoltages result {
            left_vel * m_params.left_Kv + left_accel * m_params.left_Ka,
            right_vel * m_params.right_Kv + right_accel * m_params.right_Ka
        };

        result.left_voltage +=
          units::sgn(result.left_voltage) * m_params.left_Ks;
        result.right_voltage +=
          units::sgn(result.right_voltage) * m_params.right_Ks;

        last_speeds = { left_vel, right_vel };

        return result;
    }

    VelocityControllerParams getParams() {
        return m_params;
    }

    VelocityController(VelocityControllerParams params)
        : m_params(params) {}
};

template<typename Controller>
    requires Feedforward<Controller, DifferentialSpeeds, DifferentialVoltages>
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
