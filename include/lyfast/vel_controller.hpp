#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedforward/feedforward.hpp"
#include "blazing/utils.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

class VelocityController {
    Divided<Voltage, LinearVelocity> ff_linear_vel;
    Divided<Voltage, LinearAcceleration> ff_linear_accel;
    Divided<Voltage, AngularVelocity> ff_angular_vel;
    Divided<Voltage, AngularAcceleration> ff_angular_accel;
    Voltage K_s;

    std::optional<DifferentialSpeeds> last_speeds = std::nullopt;

  public:
    DifferentialVoltages update(DifferentialSpeeds target, Time duration) {
        LinearAcceleration linear_acceleration =
          (target.linear_velocity -
           last_speeds
             // if no last speeds then we assume zero acceleration
             .value_or(target)
             .linear_velocity) /
          duration;
        AngularAcceleration angular_acceleration =
          (target.angular_velocity -
           last_speeds
             // if no last speeds then we assume zero acceleration
             .value_or(target)
             .angular_velocity) /
          duration;
        ;

        Voltage uLinear = target.linear_velocity * ff_linear_vel +
                          linear_acceleration * ff_linear_accel;
        Voltage uAngular = target.angular_velocity * ff_angular_vel +
                           angular_acceleration * ff_angular_accel;

        DifferentialVoltages result;

        result.left_voltage = uLinear - uAngular;
        result.right_voltage = uLinear + uAngular;

        result.left_voltage += units::sgn(result.left_voltage) * K_s;
        result.right_voltage += units::sgn(result.right_voltage) * K_s;

        last_speeds = target;

        return result;
    }

    VelocityController(Divided<Voltage, LinearVelocity> ff_linear_vel,
                       Divided<Voltage, LinearAcceleration> ff_linear_accel,
                       Divided<Voltage, AngularVelocity> ff_angular_vel,
                       Divided<Voltage, AngularAcceleration> ff_angular_accel,
                       Voltage K_s)
        : ff_linear_vel(ff_linear_vel),
          ff_linear_accel(ff_linear_accel),
          ff_angular_vel(ff_angular_vel),
          ff_angular_accel(ff_angular_accel),
          K_s(K_s) {}
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
