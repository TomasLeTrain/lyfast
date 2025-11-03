#pragma once

#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>

namespace blazing {
namespace lyfast {
namespace mp {
struct RobotConstraints {
    FLength track_width;
    float coeff_friction;

    FLength wheel_diameter;
    FAngularVelocity max_wheel_ang_vel;
    FMass robot_mass;
    float motor_count;

    RobotConstraints(FLength track_width,
                     float coeff_friction,
                     FLength wheel_diameter,
                     FAngularVelocity max_wheel_ang_vel,
                     FMass robot_mass,
                     float motor_count)
        : track_width(track_width),
          coeff_friction(coeff_friction),
          wheel_diameter(wheel_diameter),
          max_wheel_ang_vel(max_wheel_ang_vel),
          robot_mass(robot_mass),
          motor_count(motor_count) {}
};

struct AngularConstraints {
    FAngularVelocity max_angular_vel;
    FAngularAcceleration max_angular_accel;
    FAngularAcceleration max_angular_decel;

    AngularConstraints(AngularVelocity max_angular_vel,
                       AngularAcceleration max_angular_accel,
                       AngularAcceleration max_angular_decel)
        : max_angular_vel(max_angular_vel),
          max_angular_accel(max_angular_accel),
          max_angular_decel(max_angular_decel) {}
};

class LinearConstraints {
  public:
    LinearVelocity max_vel;
    LinearAcceleration max_accel;
    LinearAcceleration max_decel;

    LinearConstraints(LinearVelocity max_vel,
                      LinearAcceleration max_accel,
                      LinearAcceleration max_decel)
        : max_vel(max_vel),
          max_accel(max_accel),
          max_decel(max_decel) {}
};

// contains all the constraints required by the profile generator
// since some constraints might not be changed for each trajectory,
// we should have multiple ways to make the constraints
struct Constraints : public RobotConstraints,
                     LinearConstraints,
                     AngularConstraints {
    Constraints(RobotConstraints robot_constraints,
                LinearConstraints linear_constraints,
                AngularConstraints angular_constraints)
        : RobotConstraints(robot_constraints),
          LinearConstraints(linear_constraints),
          AngularConstraints(angular_constraints) {}
};
} // namespace mp
} // namespace lyfast
} // namespace blazing
