#pragma once

#include "blazing/utils.hpp"
#include "lyfast/geometry/curve.hpp"
#include "lyfast/motion_profiling/constraints.hpp"
#include "lyfast/motor_dynamics.hpp"
#include "pros/rtos.h"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <cmath>
#include <memory>
#include <vector>

// #include <arm_neon.h>
using namespace std;

namespace blazing {
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

class Trajectory {

  public:
    std::vector<FLinearVelocity> max_kin_vel_debug;
    std::vector<FLinearVelocity> max_turn_vel_debug;
    std::vector<FLinearVelocity> max_friction_vel_debug;

    std::vector<FLinearVelocity> forwards_pass_debug;
    std::vector<FLinearVelocity> backwards_pass_debug;

    std::vector<FLinearAcceleration> max_kin_accel_debug;
    std::vector<FLinearAcceleration> max_turn_accel_debug;
    std::vector<FLinearAcceleration> max_kin_decel_debug;
    std::vector<FLinearAcceleration> max_turn_decel_debug;

    std::vector<FLinearVelocity> final_vels_debug;

  private:
    geometry::Curve* curve;

    void compute() {
        auto start_time = pros::c::micros();

        // printf("total distance of curve:
        // %f\n",this->curve->total_distance.internal()); Length cd = 0_m;
        float t, previous_t = -1.0;

        for (FLength curr_dist = 0_Fm; curr_dist < curve->total_distance;
             curr_dist += delta_distance) {
            if (previous_t < 0.0)
                t = curve->t_by_s(curr_dist);
            else
                t = curve->t_by_s(curr_dist, previous_t);
            previous_t = t;

            // TODO: the s(t) is already calculated in t_by_s for cubic beziers,
            // we could return it to save some computation
            geometry::Point df = curve->df(t);

            points.emplace_back(curve->f(t),
                                curve->c(t, df),
                                df.getAngle(),
                                curve->s(t),
                                t);
        }
        // add last point
        points.emplace_back(curve->f(1),
                            curve->c(1),
                            curve->df(1).getAngle(),
                            curve->total_distance,
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

        // TODO: could be vectorized
        for (MotionPoint& point : points) {
            point.vel = units::sqrt(point.vel_squared);

            final_vels_debug.emplace_back(point.vel);
        }

        printf("sqrts:%llu\n", pros::c::micros() - start_time);
        setTravelTimes();
        printf("travel times:%llu\n", pros::c::micros() - start_time);

        // update angular velocities
        for (auto&& point : points) {
            point.ang_vel = Frad * point.vel * point.curvature;
        }

        printf("update angular vels/final time:%llu\n",
               pros::c::micros() - start_time);
    }

    // computes the isolated constraints
    // These constraints do not depend on any other points
    void isolatedConstraints() {
        const FLinearAcceleration friction_multiplier =
          constraints.coeff_friction * (9.81_Fmps2);

        // half the track width
        const FLength track_radius = constraints.track_width * 0.5f;

        for (MotionPoint& point : points) {
            // prevent divisions by zero
            const FCurvature abs_curvature =
              units::max(FCurvature(1e-5), units::abs(point.curvature));

            const FLength abs_radius = 1.0f / abs_curvature;
            const auto abs_radius_rad = abs_radius / Frad;

            const float kin_multiplier =
              1.0 / (1.0 + (track_radius * abs_curvature));

            const FLinearVelocity max_kin_vel =
              constraints.max_vel * kin_multiplier;
            const FLinearVelocity max_turn_vel =
              constraints.max_angular_vel * abs_radius_rad;

            point.vel = units::min(max_kin_vel, max_turn_vel);

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

            // const FLinearAcceleration max_kin_accel =
            //   constraints.max_accel * kin_multiplier;
            // const FLinearAcceleration max_turn_accel =
            //   constraints.max_angular_accel * abs_radius_rad;
            //
            // const FLinearAcceleration max_kin_decel =
            //   constraints.max_decel * kin_multiplier;
            // const FLinearAcceleration max_turn_decel =
            //   constraints.max_angular_decel * abs_radius_rad;

            // point.accel = units::min(max_kin_accel, max_turn_accel);
            // point.decel = units::min(max_kin_decel, max_turn_decel);
            point.accel = constraints.max_accel;
            point.decel = constraints.max_decel;

            max_kin_vel_debug.emplace_back(max_kin_vel);
            max_turn_vel_debug.emplace_back(max_turn_vel);
            max_kin_accel_debug.emplace_back(point.accel);
            max_kin_decel_debug.emplace_back(point.decel);
            //
            // max_kin_accel_debug.emplace_back(max_kin_accel);
            // max_turn_accel_debug.emplace_back(max_turn_accel);
            // max_kin_decel_debug.emplace_back(max_kin_decel);
            // max_turn_decel_debug.emplace_back(max_turn_decel);
            max_friction_vel_debug.emplace_back(units::sqrt(point.vel_squared));
        }

        // sets the start and initial velocity constraints
        points.front().vel = start_vel;
        points.back().vel = end_vel;

        // TODO: add constrain points which get minned with the max's

        // sets points.vel_squared to the right values
        for (MotionPoint& point : points) {
            point.vel_squared =
              units::min(point.vel * point.vel, point.vel_squared);
        }

        // printf("vector size: %d\n", this->points.size());
    }

    // performs a forward pass to keep max acceleration constraints
    void forwardsPass() {
        // constant for all points
        const Length dd2_multiplier = 2 * delta_distance;

        // excludes starting point
        for (size_t i = 1; i < points.size(); i++) {
            const MotionPoint& last_point = points[i - 1];
            MotionPoint& point = points[i];
            // (Sprunk 25)

            const FLength wheel_diameter = 3.25_in;
            const FMass robot_mass = 13_lb;

            const FLinearVelocity last_vel =
              units::sqrt(last_point.vel_squared);
            const FAngularVelocity last_wheel_ang_vel =
              rad * last_vel / (wheel_diameter * 2 * M_PI);

            const FTorque curr_torque =
              motor_torque(last_wheel_ang_vel / 450_rpm);

            // torque = F * r
            // torque = (m * a) * r
            // torque / (m * r) = a

            const float motor_count = 6.0f;

            const FLinearAcceleration curr_accel =
              curr_torque * motor_count / (wheel_diameter * robot_mass);

            // const FLinearAcceleration curr_accel =
            //   last_vel < 0.92085_mps ?
            //     10.51319_mps2 :
            //     last_vel * (-10.0708 / sec) + 19.786906424_mps2;

            // max velocity squared
            const auto max_vel_squared =
              // last_point.vel_squared + last_point.accel * dd2_multiplier;
              last_point.vel_squared + curr_accel * dd2_multiplier;

            // keep minimum of current max vel and previous max vel
            point.vel_squared = units::min(point.vel_squared, max_vel_squared);

            forwards_pass_debug.emplace_back(units::sqrt(point.vel_squared));
        }
    }

    // performs a forward pass to keep max deceleration constraints
    void backwardsPass() {
        // constant for all points
        const Length dd2_multiplier = 2 * delta_distance;
        // excludes end point
        for (int i = this->points.size() - 2; i >= 0; i--) {
            // (Sprunk 25)
            MotionPoint& point = points[i];
            const MotionPoint& next_point = points[i + 1];

            // decel from current point used since the motion still goes
            // forwards (the decel starts from the current point)
            const auto max_vel_squared =
              next_point.vel_squared + point.decel * dd2_multiplier;

            // keep minimum of current max vel and previous max vel
            point.vel_squared = units::min(point.vel_squared, max_vel_squared);

            // reversed:
            backwards_pass_debug.insert(backwards_pass_debug.begin(),
                                        units::sqrt(point.vel_squared));
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
    Constraints constraints;
    FLinearVelocity start_vel, end_vel;

    std::vector<MotionPoint> points;

    // change in distance between points
    FLength delta_distance;

    // total time that the motion should take
    FTime travel_time;

    DifferentialSpeeds get_by_time(Time time) {
        // actual values of the point don't really matter
        // (except for travel_time)
        MotionPoint query_point = points[0];

        query_point.travel_time = time;

        auto travel_time_cmp = [](const MotionPoint& lhs,
                                  const MotionPoint& rhs) -> bool {
            return lhs.travel_time < rhs.travel_time;
        };

        // search for time in points
        auto result_itr = lower_bound(points.begin(),
                                      points.end(),
                                      query_point,
                                      travel_time_cmp);

        if (result_itr == points.end()) {
            return { points.back().vel, points.back().ang_vel };
        } else {
            return { result_itr->vel, result_itr->ang_vel };
        }
    }

    Time getTotalTime() {
        return points.back().travel_time;
    }

    Trajectory(geometry::Curve* curve,
               Constraints constraints,
               LinearVelocity start_vel,
               LinearVelocity end_vel,
               Length change_in_distance)
        : curve(curve),
          constraints(constraints),
          start_vel(start_vel),
          end_vel(end_vel),
          delta_distance(change_in_distance) {

        travel_time = 0_sec;

        printf("total distance: %f\n", curve->total_distance.convert(in));
        printf("delta_distance: %f\n", delta_distance.convert(in));
        printf("allocated for points %d\n",
               static_cast<size_t>(
                 (curve->total_distance / delta_distance).internal()) +
                 10);
        // makes the creation of points faster by allocating the required space
        points.reserve(static_cast<size_t>(
                         (curve->total_distance / delta_distance).internal()) +
                       10);
        compute();
    }
};
} // namespace mp
} // namespace lyfast
} // namespace blazing
