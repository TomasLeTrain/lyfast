#pragma once

#include "lyfast/motion_profiling/constraints.h"
#include "lyfast/geometry/curve.h"
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

namespace mp {
class MotionPoint {
  public:
    Point point;
    Curvature curvature;
    // also used for following the trajectory
    Angle heading;
    Length arc_length;

    LinearVelocity vel = static_cast<LinearVelocity>(infinity());
    AngularVelocity ang_vel = static_cast<AngularVelocity>(infinity());

    LinearAcceleration accel = static_cast<LinearAcceleration>(infinity());
    LinearAcceleration decel = static_cast<LinearAcceleration>(infinity());

    Exponentiated<LinearVelocity, std::ratio<2>> vel2 =
      static_cast<Exponentiated<LinearVelocity, std::ratio<2>>>(infinity());

    float spline_time;
    Time travel_time = -1_sec;

    MotionPoint(Point point,
                Curvature curvature,
                // only used for getting heading
                Angle heading,
                Length arc_length,

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
    std::unique_ptr<Curve> curve;

    void compute() {
        int start_time = pros::c::micros();
        // printf("total distance of curve:
        // %f\n",this->curve->total_distance.internal()); Length cd = 0_m;
        for (Length cd = 0_m; cd < this->curve->total_distance;
             cd += this->delta_distance) {
            const float t = this->curve->t_by_dist(cd);

            // cout << "t: " << t << '\n';
            // cout << "cd: " << cd << ", dist_by_t(t_by_dist(cd)): " <<
            // this->curve->dist_by_t(t) << '\n';

            this->points.emplace_back(this->curve->f(t),
                                      this->curve->c(t),
                                      this->curve->df(t).theta(),
                                      this->curve->dist_by_t(t),
                                      t);
        }
        // add last point
        this->points.emplace_back(this->curve->f(1),
                                  this->curve->c(1),
                                  this->curve->df(1).theta(),
                                  this->curve->dist_by_t(1),
                                  1);
        printf("adding points:%llu\n", pros::c::micros() - start_time);
        this->isolatedConstraints();
        printf("isolated constraints:%llu\n", pros::c::micros() - start_time);
        this->forwardsPass();
        printf("forwards pass:%llu\n", pros::c::micros() - start_time);
        this->backwardsPass();
        printf("backwards pass:%llu\n", pros::c::micros() - start_time);

        // here we want to update vel, as after both passes we only kept vel2 up
        // to date we also dont have to take the min of point.vel and point.vel2
        // since point.vel2 has the final velocity of each particle from both
        // passes

        // hopefully this is vectorized
        for (auto&& point : this->points) {
            point.vel = units::sqrt(point.vel2);
        }

        printf("sqrts:%llu\n", pros::c::micros() - start_time);
        this->setTravelTimes();
        printf("travel times:%llu\n", pros::c::micros() - start_time);

        // update angular velocities
        for (auto&& point : points) {
            point.ang_vel = rad * point.vel * point.curvature;
        }
        printf("update angular vels/final time:%llu\n",
               pros::c::micros() - start_time);
    }

    void isolatedConstraints() {
        const LinearAcceleration friction_multiplier =
          this->constraints->coeff_friction * (9.81_mps2);

        for (auto&& point : this->points) {
            const Curvature abs_curvature = units::abs(point.curvature);
            const Length abs_radius = 1.0 / abs_curvature;
            const auto abs_radius_rad = abs_radius / rad;
            const Number kin_multiplier =
              2.0 / (this->constraints->track_width * abs_curvature + 2.0);

            const LinearVelocity max_kin_vel =
              this->constraints->max_vel * kin_multiplier;
            const LinearVelocity max_turn_vel =
              this->constraints->max_angular_vel * abs_radius_rad;
            point.vel = units::min(max_kin_vel, max_turn_vel);

            if (point.curvature.internal() != 0) {
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
                point.vel2 = friction_multiplier * abs_radius;
            }

            const LinearAcceleration max_kin_accel =
              this->constraints->max_accel * kin_multiplier;
            const LinearAcceleration max_turn_accel =
              this->constraints->max_angular_accel * abs_radius_rad;

            const LinearAcceleration max_kin_decel =
              this->constraints->max_decel * kin_multiplier;
            const LinearAcceleration max_turn_decel =
              this->constraints->max_angular_decel * abs_radius_rad;

            point.accel = units::min(max_turn_accel, max_kin_accel);
            point.decel = units::min(max_turn_decel, max_kin_decel);
        }
        this->points.front().vel = this->start_vel;
        this->points.back().vel = this->end_vel;

        // sets points.vel2 to the right values
        for (auto&& point : this->points) {
            point.vel2 = units::min(point.vel * point.vel, point.vel2);
        }

        printf("vector size: %d\n", this->points.size());
    }

    void forwardsPass() {
        const Length dd2_multiplier = 2 * this->delta_distance;
        // excludes starting point
        for (size_t i = 1; i < this->points.size(); i++) {
            const MotionPoint* last_point = &this->points[i - 1];
            // (Sprunk 25)
            // const LinearVelocity max_vel = units::sqrt(
            //         last_point->vel * last_point->vel + last_point->accel *
            //         dd2_multiplier);
            // this->points[i].vel = units::min(this->points[i].vel, max_vel);

            const auto max_vel2 =
              last_point->vel2 + last_point->accel * dd2_multiplier;
            this->points[i].vel2 = units::min(this->points[i].vel2, max_vel2);
        }
    }

    void backwardsPass() {
        const Length dd2_multiplier = 2 * this->delta_distance;
        // excludes end point
        for (int i = this->points.size() - 2; i >= 0; i--) {
            const MotionPoint* point = &this->points[i];
            const MotionPoint* next_point = &this->points[i + 1];
            // (Sprunk 25)
            // we want to decelerate from point to next_point, so we use decel
            // from point instead of next_point const LinearVelocity max_vel =
            // units::sqrt(
            //         next_point->vel * next_point->vel + point->decel *
            //         dd2_multiplier);
            // this->points[i].vel = units::min(this->points[i].vel, max_vel);

            const auto max_vel2 =
              next_point->vel2 + point->decel * dd2_multiplier;
            this->points[i].vel2 = units::min(this->points[i].vel2, max_vel2);
            // printf("%d\n",i);
        }
    }

    void setTravelTimes() {
        const Length delta_distance2 = 2 * this->delta_distance;
        this->points[0].travel_time = 0_sec;
        for (size_t i = 1; i < this->points.size() - 1; i++) {
            const MotionPoint* point = &this->points[i];
            const MotionPoint* last_point = &this->points[i - 1];
            // (Sprunk 23)
            const Time d_travel_time =
              (delta_distance2) / (point->vel + last_point->vel);
            this->points[i].travel_time =
              last_point->travel_time + d_travel_time;
        }
        // update the total travel_time
        this->travel_time = this->points[this->points.size() - 1].travel_time;
    }

  public:
    LinearVelocity start_vel, end_vel;
    Constraints* constraints;

    std::vector<MotionPoint> points;

    /**
     * @brief change in distance between points
     */
    Length delta_distance;
    /**
     * @brief inverse of delta_distance
     */
    Curvature density;
    /**
     * @brief total Time that the motion should take 
     */
    Time travel_time;

    template<typename T,
             typename = std::enable_if_t<std::is_base_of_v<Curve, T>>>
    Trajectory(T&& curve,
               Constraints* constraints,
               LinearVelocity start_vel,
               LinearVelocity end_vel,
               Length change_in_distance)
        : curve(std::make_unique<T>(std::move(curve))),
          constraints(constraints),
          start_vel(start_vel),
          end_vel(end_vel),
          delta_distance(change_in_distance) {

        this->density = 1.0 / this->delta_distance;
        this->travel_time = 0_sec;

        printf("total distance: %f\n", this->curve->total_distance.convert(in));
        printf("this->dd: %f\n", this->delta_distance.convert(in));
        // makes the creation of points faster by allocating the required space
        this->points.reserve(
          static_cast<size_t>(
            (this->curve->total_distance / this->delta_distance).internal()) +
          10);
        compute();
    }
};
} // namespace mp
