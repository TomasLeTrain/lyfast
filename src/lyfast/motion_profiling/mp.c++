#include "lyfast/motion_profiling/mp.hpp"
#include "lyfast/geometry/curve.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <algorithm>
#include <iterator>
#include <memory>
#include <variant>

namespace blazing {
namespace lyfast {
namespace mp {

std::shared_ptr<geometry::Curve> Trajectory::getCurve() {
    return curve;
}

void Trajectory::compute() {
    auto start_time = pros::c::micros();

    float t = 0;

    for (FLength curr_dist = 0_Fm; curr_dist < curve->total_distance;
         curr_dist += delta_distance) {

        // reused for getting heading and calculating curvature
        geometry::Point df = curve->df(t);

        points.emplace_back(curve->f(t),
                            curve->c(t, df),
                            units::constrainAngle2pi(df.getAngle()),
                            curr_dist,
                            t);

        // dt * v = d
        // dt = d / v
        // t += dt
        float delta_t = delta_distance / df.magnitude();
        // use std::min since units min casts to Number (doubles)
        t = std::min(t + delta_t, 1.0f);
    }
    // add last point
    points.emplace_back(curve->f(1),
                        curve->c(1),
                        units::constrainAngle2pi(curve->df(1).getAngle()),
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

        // left and right velocties cannot go above v_max:
        // l = v - w * tr <= v_max
        // r = v + w * tr <= v_max
        //
        // l = v - (v * c) * tr <= v_max
        // r = v + (v * c) * tr <= v_max
        //
        // l = v(1 - c * tr) <= v_max
        // r = v(1 + c * tr) <= v_max
        //
        // v = v_max / (1 - c * tr)
        // v = v_max / (1 + c * tr)
        //
        // v = v_max / max(1 - c * tr, 1 + c * tr)
        // v = v_max / (1 + |c|*tr)
        // ->
        // kin_multiplier = 1 / (1 + |c|*tr)
        // v = v_max * kin_multiplier

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

    // TODO: add possibility of different constraints:
    // (max/min vel/ang_vel/decel/accel for some range range)

    for (PointConstraint& constraint : point_constraints) {
        // finds closest point to constraint and sets those values
        float spline_time = 0;

        if (std::holds_alternative<float>(constraint.timeframe)) {
            // indexed by spline time
            spline_time = units::clamp(get<float>(constraint.timeframe), 0, 1);
        } else if (std::holds_alternative<FLength>(constraint.timeframe)) {
            // indexed by distance
            std::cout << "using distance "
                      << get<FLength>(constraint.timeframe).convert(in)
                      << std::endl;
            FLength distance = units::min(curve->total_distance,
                                          get<FLength>(constraint.timeframe));
            spline_time = curve->t_by_s(distance);
        }

        FLength dist = curve->s(spline_time);
        int index =
          int(float(dist / curve->total_distance) * (points.size() - 1));

        std::cout << "constraint from " << index << std::endl;

        std::cout << "before vel/accel/decel: "
                  << points[index].vel.convert(inps) << " "
                  << points[index].accel.convert(inps2) << " "
                  << points[index].decel.convert(inps2) << std::endl;

        // apply constraints
        if (constraint.vel)
            points[index].vel = units::min(points[index].vel, *constraint.vel);

        if (constraint.accel)
            points[index].accel =
              units::min(points[index].accel, *constraint.accel);

        if (constraint.decel)
            points[index].decel =
              units::min(points[index].decel, *constraint.decel);

        std::cout << "after vel/accel/decel: "
                  << points[index].vel.convert(inps) << " "
                  << points[index].accel.convert(inps2) << " "
                  << points[index].decel.convert(inps2) << std::endl;
    }

    // sets the start and initial velocity constraints
    points.front().vel = start_vel;
    points.back().vel = end_vel;
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
    const FLength distance_delta_mult = 2 * delta_distance;
    forwards_pass_debug.emplace_back(points.front().vel);

    // excludes starting point
    for (size_t i = 1; i < points.size(); i++) {
        const MotionPoint& last_point = points[i - 1];
        MotionPoint& point = points[i];
        // (Sprunk 25)

        // predicted max accel from motors
        const FLinearAcceleration motor_accel =
          get_accel(units::abs(last_point.vel));

        const FLinearAcceleration accel =
          units::min(motor_accel, last_point.accel);

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
    const FLength distance_delta_mult = 2 * delta_distance;
    backwards_pass_debug.insert(backwards_pass_debug.begin(),
                                points.back().vel);

    // excludes end point
    for (int i = this->points.size() - 2; i >= 0; i--) {
        // (Sprunk 25)
        MotionPoint& point = points[i];
        const MotionPoint& next_point = points[i + 1];

        // predicted max decel from motors
        const FLinearAcceleration motor_accel =
          // sort of predicting backwards here, but we don't know what point's
          // vel will be
          get_accel(units::abs(next_point.vel));

        // decel from current point used since the motion still goes
        // forwards (the decel starts from the current point)
        const FLinearAcceleration decel = units::min(motor_accel, point.decel);

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

        // in case vel is negative the delta distance would also be positive, so
        // this term should always be positive
        const Time delta_travel_time =
          units::abs((delta_distance2) / (point.vel + last_point.vel));

        point.travel_time = last_point.travel_time + delta_travel_time;
    }
}

int Trajectory::indexByDistance(FLength distance, int start_ind) {
    // actual values of the point don't really matter
    // (except for arc_length)
    MotionPoint query_point = points[0];

    query_point.arc_length = distance;

    int idx_guess = std::round(distance / delta_distance);

    const int num_points = getNumPoints();

    // adjust start_ind to avoid lo > hi clamp undefined behavior
    if (start_ind >= num_points) {
        start_ind = num_points - 1;
    }

    return std::clamp(idx_guess, start_ind, num_points - 1);
};

int Trajectory::indexByTime(FTime time, int start_ind) {
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

    int idx = std::distance(points.begin(), result_itr);

    const int num_points = getNumPoints();

    // adjust start_ind to avoid lo > hi clamp undefined behavior
    if (start_ind >= num_points) {
        start_ind = num_points - 1;
    }

    return std::clamp(idx, start_ind, num_points - 1);
};

int Trajectory::indexByClosestPoint(geometry::Point point,
                                    int start_ind,
                                    FLength max_look_dist,
                                    FLength resolution) {
    FLength best = Length(INFINITY);
    int result = 0;

    FLength original_start_dist = points[start_ind].arc_length;
    // either some max look dist or look until the end of the array
    FLength original_end_dist =
      units::min(original_start_dist + max_look_dist, getTotalDistance());

    FLength start_dist = original_start_dist;
    FLength end_dist = original_end_dist;

    for (Length curr_dist = start_dist; curr_dist <= end_dist;
         curr_dist += resolution) {
        // get index by distance
        int i = indexByDistance(curr_dist);

        auto& motion_point = points[i];
        Length curr_distance = point.distanceTo(motion_point.point);
        if (curr_distance < best) {
            best = curr_distance;
            result = i;
        }
    }

    // here result is likely close to optimal, but we can run second loop to
    // find closest one
    // the clamping makes sure we don't bypass the set constraints
    start_dist = units::clamp(getPoint(result).arc_length - resolution,
                              original_start_dist,
                              original_end_dist);
    end_dist = units::clamp(getPoint(result).arc_length + resolution,
                            original_start_dist,
                            original_end_dist);

    // search with 10x resolution in small search space around found solution
    for (Length curr_dist = start_dist; curr_dist <= end_dist;
         curr_dist += resolution / 10.0) {
        // get index by distance
        int i = indexByDistance(curr_dist);

        auto& motion_point = points[i];
        Length curr_distance = point.distanceTo(motion_point.point);
        if (curr_distance < best) {
            best = curr_distance;
            result = i;
        }
    }

    // here result is very likely optimal
    return result;
}

int Trajectory::sanitizeIndex(int index) {
    return std::clamp(index, 0, int(getNumPoints()));
}

FDifferentialSpeeds Trajectory::differentialVelocitiesByIndex(int index) {
    auto& point = getPoint(index);

    return { point.vel, Frad * point.vel * point.curvature };
}

FLength Trajectory::getTotalDistance() {
    return points.back().arc_length;
}

FTime Trajectory::getTotalTime() {
    return points.back().travel_time;
}

size_t Trajectory::getNumPoints() {
    return points.size();
}

MotionPoint& Trajectory::getPoint(int index) {
    return points.at(index);
}

MotionPoint& Trajectory::getMotionEndPoint() {
    return points.front();
}

MotionPoint& Trajectory::getMotionStartPoint() {
    return points.back();
}

Trajectory::Trajectory(std::shared_ptr<geometry::Curve> curve,
                       Constraints constraints,
                       std::vector<PointConstraint> point_constraints,
                       LinearVelocity start_vel,
                       LinearVelocity end_vel,
                       Length change_in_distance)
    : curve(curve),
      constraints(constraints),
      start_vel(start_vel),
      end_vel(end_vel),
      delta_distance(change_in_distance),
      point_constraints(point_constraints) {

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
