#pragma once

#include "lyfast/geometry/curve.h"
#include "lyfast/motion_profiling/constraints.h"
#include "pros/rtos.h"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <arm_neon.h>
#include <array>
#include <cmath>
#include <memory>
#include <vector>

// #include <arm_neon.h>
using namespace std;

namespace lyfast {
namespace mp {

struct MotionPoint {
    geometry::Point point;
    FCurvature curvature;
    FAngle heading;
    FLength arc_length;

    FLinearVelocity vel = LinearVelocity(infinity());
    FAngularVelocity ang_vel = AngularVelocity(infinity());

    FLinearAcceleration accel = LinearAcceleration(infinity());
    FLinearAcceleration decel = LinearAcceleration(infinity());

    // intermediate velocity squared
    Exponentiated<FLinearVelocity, std::ratio<2>> vel_squared =
      Exponentiated<FLinearVelocity, std::ratio<2>>(infinity());

    float spline_time;
    FTime travel_time = -1_sec;

    MotionPoint(geometry::Point point,
                FCurvature curvature,
                FAngle heading,
                FLength arc_length,
                float spline_time)
        : point(point),
          curvature(curvature),
          heading(heading),
          arc_length(arc_length),
          spline_time(spline_time) {}
};

// need to make classes for:
// path segment
// trajectory generator
// trajectory
//
// some sort of generator that makes a trajectory given a set of curves and
// constraints

// each trajectory should be a continous curve
// this means the trajectory generator takes only one curve as input
class Trajectory {
  private:
    geometry::Curve* curve;

    void compute() {
        auto start_time = pros::c::micros();

        // printf("total distance of curve:
        // %f\n",this->curve->total_distance.internal()); Length cd = 0_m;
        float t, previous_t = -1.0;
        for (FLength curr_dist = 0_Fm; curr_dist < curve->s(1.0);
             curr_dist += delta_distance) {
            if (previous_t < 0.0)
                t = curve->t_by_s(curr_dist);
            else
                t = curve->t_by_s(curr_dist, previous_t);
            previous_t = t;

            points.emplace_back(curve->f(t),
                                curve->c(t),
                                curve->df(t).getAngle(),
                                curve->s(t),
                                t);
        }
        // add last point
        points.emplace_back(curve->f(1),
                            curve->c(1),
                            curve->df(1).getAngle(),
                            curve->s(1),
                            1);

        printf("adding points:%llu\n", pros::c::micros() - start_time);
        isolatedConstraints();
        printf("isolated constraints:%llu\n", pros::c::micros() - start_time);
        forwardsPass();
        printf("forwards pass:%llu\n", pros::c::micros() - start_time);
        backwardsPass();
        printf("backwards pass:%llu\n", pros::c::micros() - start_time);

        // here we want to update vel, as after both passes we only kept vel2 up
        // to date we also dont have to take the min of point.vel and point.vel2
        // since point.vel2 has the final velocity of each particle from both
        // passes

        // hopefully this is vectorized
        for (MotionPoint& point : points) {
            point.vel = units::sqrt(point.vel_squared);
        }

        printf("sqrts:%llu\n", pros::c::micros() - start_time);
        setTravelTimes();
        printf("travel times:%llu\n", pros::c::micros() - start_time);

        // update angular velocities
        for (auto&& point : points) {
            point.ang_vel = rad * point.vel * point.curvature;
        }
        printf("update angular vels/final time:%llu\n",
               pros::c::micros() - start_time);
    }

    // computes the isolated constraints
    // These constraints do not depend on any other points
    void isolatedConstraints() {
        const FLinearAcceleration friction_multiplier =
          constraints.coeff_friction * (9.81_Fmps2);

        for (MotionPoint& point : points) {
            const FCurvature abs_curvature = units::abs(point.curvature);
            const FLength abs_radius = 1.0f / abs_curvature;
            const auto abs_radius_rad = abs_radius / Frad;
            const float kin_multiplier =
              2.0f / (constraints.track_width * abs_curvature + 2.0f);

            const FLinearVelocity max_kin_vel =
              constraints.max_vel * kin_multiplier;
            const FLinearVelocity max_turn_vel =
              constraints.max_angular_vel * abs_radius_rad;

            // still considers the current point velocity in case it was set
            // before as a constraint
            point.vel = units::min(point.vel, max_kin_vel);
            point.vel = units::min(point.vel, max_turn_vel);

            if (units::abs(point.curvature).internal() > 1e-6) {
                // Ff = Fg * coeff_friction -> Ff = m * g * coeff_friction

                // ac = v^2 / r -> ac = v^2 * |c|
                // Ff = m * ac  -> Ff = m * v^2 * |c|

                // m * v^2 * |c| = m * g * coeff_friction
                // v^2 * |c| = g * coeff_friction
                // v = sqrt(g * coeff_friction / |c|)

                // const LinearVelocity max_slip_vel =
                //     units::sqrt(
                //         friction_multiplier * abs_radius
                //     );
                // point.vel = units::min(point.vel, max_slip_vel);
                point.vel_squared = friction_multiplier * abs_radius;
            }

            const FLinearAcceleration max_kin_accel =
              constraints.max_accel * kin_multiplier;
            const FLinearAcceleration max_turn_accel =
              constraints.max_angular_accel * abs_radius_rad;

            const FLinearAcceleration max_kin_decel =
              constraints.max_decel * kin_multiplier;
            const FLinearAcceleration max_turn_decel =
              constraints.max_angular_decel * abs_radius_rad;

            point.accel = units::min(max_turn_accel, max_kin_accel);
            point.decel = units::min(max_turn_decel, max_kin_decel);
        }
        points.front().vel = start_vel;
        points.back().vel = end_vel;

        // sets points.vel2 to the right values
        for (MotionPoint& point : points) {
            point.vel_squared =
              units::min(point.vel * point.vel, point.vel_squared);
        }

        printf("vector size: %d\n", this->points.size());
    }

    // performs a forward pass to keep max acceleration constraints
    void forwardsPass() {
        // constant for all points
        const Length dd2_multiplier = 2 * delta_distance;

        // excludes starting point
        for (size_t i = 1; i < points.size(); i++) {
            const MotionPoint& last_point = points[i - 1];
            // (Sprunk 25)

            // max velocity squared
            const auto max_vel2 =
              last_point.vel_squared + last_point.accel * dd2_multiplier;

            // keep minimum of current max vel and previous max vel
            points[i].vel_squared = units::min(points[i].vel_squared, max_vel2);
        }
    }

    // performs a forward pass to keep max deceleration constraints
    void backwardsPass() {
        // constant for all points
        const Length dd2_multiplier = 2 * delta_distance;
        // excludes end point
        for (int i = this->points.size() - 2; i >= 0; i--) {
            // (Sprunk 25)
            const MotionPoint& point = points[i];
            const MotionPoint& next_point = points[i + 1];

            // uses velocity from next point (before in the motion) but decel
            // from current point
            const auto max_vel2 =
              next_point.vel_squared + point.decel * dd2_multiplier;

            // keep minimum of current max vel and previous max vel
            points[i].vel_squared = units::min(points[i].vel_squared, max_vel2);
        }
    }

    void setTravelTimes() {
        const FLength delta_distance2 = 2 * delta_distance;
        points[0].travel_time = 0_sec;

        for (size_t i = 1; i < points.size(); i++) {
            const MotionPoint& point = points[i];
            const MotionPoint& last_point = points[i - 1];
            // (Sprunk 23)
            const Time delta_travel_time =
              (delta_distance2) / (point.vel + last_point.vel);

            points[i].travel_time = last_point.travel_time + delta_travel_time;
        }
        // update the total travel_time
        travel_time = points.back().travel_time;
    }

  public:
    FLinearVelocity start_vel, end_vel;
    Constraints constraints;

    std::vector<MotionPoint> points;

    // change in distance between points
    FLength delta_distance;

    // total time that the motion should take
    FTime travel_time;

    template<typename T,
             typename = std::enable_if_t<std::is_base_of_v<geometry::Curve, T>>>
    Trajectory(T&& curve,
               Constraints constraints,
               LinearVelocity start_vel,
               LinearVelocity end_vel,
               Length change_in_distance)
        : curve(std::make_unique<T>(std::move(curve))),
          constraints(constraints),
          start_vel(start_vel),
          end_vel(end_vel),
          delta_distance(change_in_distance) {

        travel_time = 0_sec;

        printf("total distance: %f\n", curve->s(1.0).convert(in));
        printf("this->dd: %f\n", this->delta_distance.convert(in));
        // makes the creation of points faster by allocating the required space
        this->points.reserve(
          static_cast<size_t>(
            (curve->s(1.0) / this->delta_distance).internal()) +
          10);
        compute();
    }
};
} // namespace mp
} // namespace lyfast
