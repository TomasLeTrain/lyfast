#include "lyfast/motion_profiling/mp.hpp"

namespace blazing {
namespace lyfast {
namespace mp {

void Trajectory::compute() {
    auto start_time = pros::c::micros();

    // printf("total distance of curve:
    // %f\n",this->curve->total_distance.internal()); Length cd = 0_m;
    // float t, previous_t = -1.0;
    float t = 0;

    for (FLength curr_dist = 0_Fm; curr_dist < curve->total_distance;
         curr_dist += delta_distance) {
        // if (previous_t < 0.0)
        //     t = curve->t_by_s(curr_dist);
        // else
        //     t = curve->t_by_s(curr_dist, previous_t);
        // previous_t = t;

        // reused for getting heading and calculating curvature
        geometry::Point df = curve->df(t);

        points.emplace_back(curve->f(t),
                            curve->c(t, df),
                            df.getAngle(),
                            // curve->s(t),
                            curr_dist,
                            t);

        // t = t + dt
        float delta_t = delta_distance / df.magnitude();
        t = units::min(t + delta_t, 1.0);
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

    for (MotionPoint& point : points) {
        final_vels_debug.emplace_back(point.vel);
    }

    setTravelTimes();
    printf("travel times:%llu\n", pros::c::micros() - start_time);

    printf("final time:%llu\n", pros::c::micros() - start_time);
}

// computes the isolated constraints
// These constraints do not depend on any other points
void Trajectory::isolatedConstraints() {
    const FLinearAcceleration friction_multiplier =
      constraints.coeff_friction * (9.81_Fmps2);

    // half the track width
    const FLength track_radius = constraints.track_width * 0.5f;

    for (MotionPoint& point : points) {
        // prevent divisions by zero
        const FCurvature abs_curvature =
          units::max(FCurvature(1e-5f), units::abs(point.curvature));

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

        // ac = v^2 / r
        // Ff = m * ac -> Ff = m * v^2 / r

        // m * v^2 * / r = m * g * coeff_friction
        // v^2 / r = g * coeff_friction
        // v = sqrt(g * coeff_friction * r)
        point.vel =
          units::min(point.vel, units::sqrt(friction_multiplier * abs_radius));

        const FLinearAcceleration max_kin_accel =
          constraints.max_accel * kin_multiplier;
        const FLinearAcceleration max_turn_accel =
          constraints.max_angular_accel * abs_radius_rad;

        const FLinearAcceleration max_kin_decel =
          constraints.max_decel * kin_multiplier;
        const FLinearAcceleration max_turn_decel =
          constraints.max_angular_decel * abs_radius_rad;

        point.accel = units::min(max_kin_accel, max_turn_accel);
        point.decel = units::min(max_kin_decel, max_turn_decel);
        // point.accel = constraints.max_accel;
        // point.decel = constraints.max_decel;

        max_kin_vel_debug.emplace_back(max_kin_vel);
        max_turn_vel_debug.emplace_back(max_turn_vel);
        max_kin_accel_debug.emplace_back(point.accel);
        max_kin_decel_debug.emplace_back(point.decel);
        //
        // max_kin_accel_debug.emplace_back(max_kin_accel);
        // max_turn_accel_debug.emplace_back(max_turn_accel);
        // max_kin_decel_debug.emplace_back(max_kin_decel);
        // max_turn_decel_debug.emplace_back(max_turn_decel);
        max_friction_vel_debug.emplace_back(point.vel);
    }

    // sets the start and initial velocity constraints
    points.front().vel = start_vel;
    points.back().vel = end_vel;

    // TODO: add possibility of different constraints:
    // (max/min vel/ang_vel/decel/accel at any range/point)
}

FLinearAcceleration Trajectory::get_accel(FLinearVelocity last_vel) {
    const FLength wheel_radius = constraints.wheel_diameter / 2.0f;

    const FAngularVelocity last_wheel_ang_vel = rad * (last_vel / wheel_radius);

    const FTorque curr_torque =
      motor_torque(last_wheel_ang_vel / constraints.max_wheel_ang_vel);

    // Torque = Force * Radius = (Mass * Accel) * Radius
    // ->
    // Accel = Torque / (Mass * Radius)
    const FLinearAcceleration curr_accel =
      (curr_torque * constraints.motor_count) /
      (wheel_radius * constraints.robot_mass);

    return curr_accel;
}

// performs a forward pass to keep max acceleration constraints
void Trajectory::forwardsPass() {
    // constant for all points
    const Length distance_delta_mult = 2 * delta_distance;

    // excludes starting point
    for (size_t i = 1; i < points.size(); i++) {
        const MotionPoint& last_point = points[i - 1];
        MotionPoint& point = points[i];
        // (Sprunk 25)

        const FLinearAcceleration accel =
          units::min(get_accel(last_point.vel), last_point.accel);

        // max velocity squared
        const FLinearVelocity max_vel = units::sqrt(
          units::square(last_point.vel) + accel * distance_delta_mult);

        // keep minimum of current max vel and previous max vel
        point.vel = units::min(point.vel, max_vel);

        forwards_pass_debug.emplace_back(point.vel);
    }
}

// performs a forward pass to keep max deceleration constraints
void Trajectory::backwardsPass() {
    // constant for all points
    const Length distance_delta_mult = 2 * delta_distance;
    // excludes end point
    for (int i = this->points.size() - 2; i >= 0; i--) {
        // (Sprunk 25)
        MotionPoint& point = points[i];
        const MotionPoint& next_point = points[i + 1];

        // decel from current point used since the motion still goes
        // forwards (the decel starts from the current point)
        const FLinearAcceleration decel =
          units::min(get_accel(next_point.vel), point.decel);

        const FLinearVelocity max_vel = units::sqrt(
          units::square(next_point.vel) + decel * distance_delta_mult);

        // keep minimum of current max vel and previous max vel
        point.vel = units::min(point.vel, max_vel);

        // reversed:
        backwards_pass_debug.insert(backwards_pass_debug.begin(), point.vel);
    }
}

void Trajectory::setTravelTimes() {
    const FLength delta_distance2 = 2 * delta_distance;
    points[0].travel_time = 0_sec;

    for (size_t i = 1; i < points.size(); i++) {
        MotionPoint& point = points[i];
        const MotionPoint& last_point = points[i - 1];
        // (Sprunk 23)
        const Time delta_travel_time =
          (delta_distance2) / (point.vel + last_point.vel);

        point.travel_time = last_point.travel_time + delta_travel_time;
    }
    // update the total travel_time
    travel_time = points.back().travel_time;
}

DifferentialSpeeds Trajectory::get_by_distance(FLength distance) {
    // actual values of the point don't really matter
    // (except for arc_length)
    MotionPoint query_point = points[0];

    query_point.arc_length = distance;

    auto travel_distance_cmp = [](const MotionPoint& lhs,
                                  const MotionPoint& rhs) -> bool {
        return lhs.arc_length < rhs.arc_length;
    };

    // search for time in points
    auto result_itr = lower_bound(points.begin(),
                                  points.end(),
                                  query_point,
                                  travel_distance_cmp);

    if (result_itr == points.end()) {
        // result is last element
        return { points.back().vel,
                 Frad * points.back().vel * points.back().curvature };
    } else {
        return { result_itr->vel,
                 Frad * result_itr->vel * result_itr->curvature };
    }
}

FLength Trajectory::getTotalDistance() {
    return points.back().arc_length;
}

DifferentialSpeeds Trajectory::get_by_time(Time time) {
    // actual values of the point don't really matter
    // (except for travel_time)
    MotionPoint query_point = points[0];

    query_point.travel_time = time;

    auto travel_time_cmp = [](const MotionPoint& lhs,
                              const MotionPoint& rhs) -> bool {
        return lhs.travel_time < rhs.travel_time;
    };

    // search for time in points
    auto result_itr =
      lower_bound(points.begin(), points.end(), query_point, travel_time_cmp);

    if (result_itr == points.end()) {
        return { points.back().vel,
                 Frad * points.back().vel * points.back().curvature };
    } else {
        return { result_itr->vel,
                 Frad * result_itr->vel * result_itr->curvature };
    }
}

Time Trajectory::getTotalTime() {
    return points.back().travel_time;
}

Trajectory::Trajectory(geometry::Curve* curve,
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
    printf(
      "allocated for points %d\n",
      static_cast<size_t>((curve->total_distance / delta_distance).internal()) +
        10);
    // makes the creation of points faster by allocating the required space
    points.reserve(
      static_cast<size_t>((curve->total_distance / delta_distance).internal()) +
      10);
    compute();
}
} // namespace mp
} // namespace lyfast
} // namespace blazing
