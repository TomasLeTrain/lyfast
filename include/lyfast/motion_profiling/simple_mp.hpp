#pragma once

#include "lyfast/geometry/curve.hpp"
#include "lyfast/geometry/primitives.hpp"
#include "lyfast/motion_profiling/constraints.hpp"
#include "pros/rtos.h"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace motions {
namespace simple_mp {
class SimpleMotionPoint {
  public:
    Time timestamp;
    Length position;
    LinearVelocity velocity;
    LinearAcceleration acceleration;

    SimpleMotionPoint(Time timestamp,
                      Length position,
                      LinearVelocity velocity,
                      LinearAcceleration acceleration)
        : timestamp(timestamp),
          position(position),
          velocity(velocity),
          acceleration(acceleration) {}
};

class TrapezoidalProfileTrajectory {
  public:
    std::vector<SimpleMotionPoint> points = {
        { 0_sec, 0_m, 0_mps, 0_mps2 }
    };

    // TODO: make into constraints struct
    LinearVelocity max_vel;
    LinearAcceleration max_accel;
    LinearAcceleration max_decel;

    // target distance
    Length target;

    Time accel_time, decel_time, decel_start_time;
    Length accel_distance, decel_start_distance;

    void compute() {
        // time to get to max vel when accelerating / get to zero when
        // decelerating
        accel_time = max_vel / max_accel;
        decel_time = max_vel / max_decel;

        // Using the acceleration time, we can find the distance it would take
        // to reach max velocity
        accel_distance = 0.5 * max_accel * units::square(accel_time);
        Length decel_distance = 0.5 * max_decel * units::square(decel_time);

        // no cruise time, we have to determine at which t they meet
        if (accel_distance + decel_distance > target) {
            accel_time = units::sqrt(2 * target /
                                     (max_accel * (1 + max_accel / max_decel)));

            decel_time = accel_time * max_accel / max_decel;

            accel_distance = 0.5 * max_accel * units::square(accel_time);
            decel_distance = 0.5 * max_decel * units::square(decel_time);

            max_vel = max_accel * accel_time;
        }

        // Then calculate the cruising distance based on the distance left
        const Length cruise_distance = target - accel_distance - decel_distance;
        const Time cruise_time =
          cruise_distance / max_vel; // Divide by the velocity to get time

        // time at which we start to decelerate
        decel_start_time = accel_time + cruise_time;
        decel_start_distance = accel_distance + cruise_distance;
    }

    LinearVelocity getVelocity(Time t) {
        // we are accelerating
        if (t < accel_time) {
            return max_accel * t;
        }
        // we are crusing
        else if (t < decel_start_time) {
            return max_vel;
        }
        // if none of the above, we must be in deceleration
        else {
            return (max_vel - max_decel * (t - decel_start_time));
        }
    }

    LinearVelocity getVelocity(Length distance) {
        if (distance < accel_distance) {
            return units::sqrt(2 * distance * max_accel);
        }
        // we are crusing
        else if (distance < decel_start_distance) {
            return max_vel;
        }
        // if none of the above, we must be decelerating
        else {
            return (
              max_vel -
              units::sqrt(2 * (distance - decel_start_distance) * max_decel));
        }
    }

    TrapezoidalProfileTrajectory(LinearVelocity max_vel,
                                 LinearAcceleration max_accel,
                                 Time dt,
                                 Length target)
        : max_vel(max_vel),
          max_accel(max_accel),
          target(target) {
        compute();
    }
};
} // namespace simple_mp
} // namespace motions
