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

    RobotConstraints(FLength track_width, float coeff_friction)
        : track_width(track_width),
          coeff_friction(coeff_friction) {}
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
    Constraints(Length track_width,
                float coeff_friction,

                LinearVelocity max_vel,
                LinearAcceleration max_accel,
                LinearAcceleration max_decel,

                AngularVelocity max_angular_vel,
                AngularAcceleration max_angular_accel,
                AngularAcceleration max_angular_decel)
        : RobotConstraints(track_width, coeff_friction),
          LinearConstraints(max_vel, max_accel, max_decel),
          AngularConstraints(max_angular_vel,
                             max_angular_accel,
                             max_angular_decel) {}

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
