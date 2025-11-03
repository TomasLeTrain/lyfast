#pragma once

#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace lyfast {

struct DifferentialVoltages {
    Voltage left_voltage;
    Voltage right_voltage;
};

struct DifferentialSpeeds {
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
};

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
                       Divided<Voltage, AngularAcceleration> ff_angular_accel)
        : ff_linear_vel(ff_linear_vel),
          ff_linear_accel(ff_linear_accel),
          ff_angular_vel(ff_angular_vel),
          ff_angular_accel(ff_angular_accel) {}
};

} // namespace lyfast
