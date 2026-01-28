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
#include <cassert>
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

    // target distance
    Length target;

    LinearVelocity max_vel;
    LinearAcceleration max_accel;
    LinearAcceleration max_decel;

    Time accel_time, decel_time, steady_time, total_time;
    Length accel_distance, decel_distance, steady_distance;

    LinearVelocity max_vel_triangular;
    Length accel_dist_triangular;
    Length decel_dist_triangular;

    // for triangle profile
    Time accel_time_triangular, decel_time_triangular;

  private:
    void precompute() {
        // trapezoidal case
        accel_time = max_vel / max_accel;
        decel_time = max_vel / max_decel;

        accel_distance = 0.5 * max_accel * accel_time * accel_time;
        decel_distance = 0.5 * max_decel * decel_time * decel_time;

        steady_distance = target - (accel_distance + decel_distance);

        steady_time = steady_distance / max_vel;

        // triangular case
        decel_time_triangular = units::sqrt(
          2 * target / (units::pow<2>(max_decel) / max_accel + max_decel));
        max_vel_triangular = decel_time_triangular * max_decel;
        accel_time_triangular = max_vel_triangular / max_accel;

        accel_dist_triangular =
          0.5 * max_accel * accel_time_triangular * accel_time_triangular;
        decel_dist_triangular =
          0.5 * max_decel * decel_time_triangular * decel_time_triangular;

        if (steady_distance > 0_in) {
            total_time = accel_time + steady_time + decel_time;
        } else {
            total_time = accel_time_triangular + decel_time_triangular;
        }
    }

    LinearVelocity triangleProfile(Time t) {
        if (t < accel_time) {
            return t * max_accel;
        } else if (t < accel_time + steady_time) {
            return max_vel;
        } else {
            return max_vel - (t - (steady_time + accel_time)) * max_decel;
        }
    }

    LinearVelocity trapezoidProfile(Time t) {
        if (t < accel_time_triangular) {
            return t * max_accel;
        } else {
            return max_accel * accel_time_triangular -
                   max_decel * (t - accel_time_triangular);
        }
    }

  public:
    LinearVelocity getVelocity(Time t) {
        if (t < 0_sec || t > total_time) return 0_mps;

        if (steady_distance < 0_in) {
            return triangleProfile(t);
        } else {
            return trapezoidProfile(t);
        }
    }

    LinearVelocity getVelocity(Length distance) {
        if (distance > target || distance < 0_in) {
            return 0_mps;
        }

        // trapezoidal case
        if (steady_distance > 0_in) {
            if (distance < accel_distance) {
                return getVelocity(units::sqrt(2 * distance / max_accel));
            } else if (distance < accel_distance + steady_distance) {
                distance -= accel_distance;

                return getVelocity(accel_time + (distance / max_vel));
            } else {
                // avoid precision error in the result
                distance -= 1e-5 * m;

                LinearAcceleration a = -0.5 * max_decel;
                LinearVelocity b = max_vel;
                Length c = (accel_distance + steady_distance) - distance;

                // smallest solution
                Time result = (-b + units::sqrt(b * b - 4 * a * c)) / (2 * a);

                return getVelocity(accel_time + steady_time + result);
            }
        } else {
			// triangular case
            if (distance < accel_dist_triangular) {
                return getVelocity(units::sqrt(2 * distance / max_accel));
            } else {
                // avoid precision error in the result
                distance -= 1e-5 * m;

                LinearAcceleration a = -0.5 * max_decel;
                LinearVelocity b = max_vel_triangular;
                Length c = accel_dist_triangular - distance;

                // smallest solution
                Time result = (-b + units::sqrt(b * b - 4 * a * c)) / (2 * a);

                return getVelocity(accel_time_triangular + result);
            }
        }
    }

    TrapezoidalProfileTrajectory(LinearVelocity max_vel,
                                 LinearAcceleration max_accel,
                                 LinearAcceleration max_decel,
                                 Length target)
        : target(target),
          max_vel(max_vel),
          max_accel(max_accel),
          max_decel(max_decel) {
        precompute();
    }
};
} // namespace simple_mp
} // namespace motions
