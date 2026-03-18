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

Constraints Trajectory::getConstraints() const {
    return m_constraints;
}

FLength Trajectory::getDeltaDistance() const {
    return m_delta_distance;
}

const std::vector<Trajectory::debugInfo>& Trajectory::getDebugInfo() const {
    return m_debug_info;
}

const std::vector<MotionPoint>& Trajectory::getPoints() const {
    return m_points;
}

const std::vector<PointConstraint>& Trajectory::getPointConstraints() const {
    return m_point_constraints;
}

const std::vector<RangeConstraint>& Trajectory::getRangeConstraints() const {
    return m_range_constraints;
}

bool Trajectory::getDebugEnabled() const {
    return m_debug_enabled;
}

FLinearVelocity Trajectory::getStartVelocity() const {
    return m_start_vel;
}

FLinearVelocity Trajectory::getEndVelocity() const {
    return m_end_vel;
}

void Trajectory::compute() {
    auto start_time = pros::c::micros();

    float t = 0;

    for (FLength curr_dist = 0_Fm; curr_dist < curve->getTotalDistance();
         curr_dist += m_delta_distance) {

        // reused for getting heading and calculating curvature
        geometry::Point df = curve->df(t);

        m_points.emplace_back(curve->f(t),
                              curve->c(t, df),
                              units::constrainAngle2pi(df.getAngle()),
                              curr_dist,
                              t);

        // dt * v = d
        // dt = d / v
        // t += dt
        float delta_t = m_delta_distance / df.magnitude();
        // use std::min since units min casts to Number (doubles)
        t = std::min(t + delta_t, 1.0f);
    }
    // add last point
    m_points.emplace_back(curve->f(1),
                          curve->c(1),
                          units::constrainAngle2pi(curve->df(1).getAngle()),
                          curve->getTotalDistance(),
                          1);

    printf("adding points:%llu\n", pros::c::micros() - start_time);
    isolatedConstraints();
    printf("isolated constraints:%llu\n", pros::c::micros() - start_time);
    forwardsPass();
    printf("forwards pass:%llu\n", pros::c::micros() - start_time);
    backwardsPass();
    printf("backwards pass:%llu\n", pros::c::micros() - start_time);

    for (size_t i = 0; i < getNumPoints(); i++) {
        m_debug_info[i].final_vels = getPoint(i).vel;
    }

    setTravelTimes();
    printf("travel times:%llu\n", pros::c::micros() - start_time);

    printf("final time:%llu\n", pros::c::micros() - start_time);
}

// finds an index given a constraint keyframe (spline time or length)
size_t Trajectory::indexByKeyframe(std::variant<float, FLength>& keyframe) {
    float spline_time = 0;
    FLength target_dist;

    if (std::holds_alternative<float>(keyframe)) {
        // indexed by spline time
        // first clamp to be in correct range
        spline_time = units::clamp(get<float>(keyframe), 0, 1);

        // calculate distance from the spline time
        target_dist = curve->s(spline_time);
    } else if (std::holds_alternative<FLength>(keyframe)) {
        // indexed by distance
        target_dist = get<FLength>(keyframe);
    }

    // find index from the distance
    return indexByDistance(target_dist);
}

void Trajectory::applyPointAndRangeConstraints() {
    for (PointConstraint& constraint : m_point_constraints) {
        size_t index = indexByKeyframe(constraint.keyframe);
        MotionPoint& curr_point = m_points[index];

        // apply constraints
        if (constraint.vel.has_value())
            curr_point.vel = units::min(curr_point.vel, *constraint.vel);

        if (constraint.accel.has_value())
            curr_point.accel = units::min(curr_point.accel, *constraint.accel);

        if (constraint.decel.has_value())
            curr_point.decel = units::min(curr_point.decel, *constraint.decel);
    }

    for (RangeConstraint& constraint : m_range_constraints) {
        size_t left_spline_idx = indexByKeyframe(constraint.left_keyframe);
        size_t right_spline_idx = indexByKeyframe(constraint.right_keyframe);

        // apply the constraints to all the points in the range
        for (size_t i = left_spline_idx; i <= right_spline_idx; i++) {
            // apply constraints
            MotionPoint& curr_point = m_points[i];

            // apply constraints
            if (constraint.vel.has_value())
                curr_point.vel = units::min(curr_point.vel, *constraint.vel);

            if (constraint.accel.has_value())
                curr_point.accel =
                  units::min(curr_point.accel, *constraint.accel);

            if (constraint.decel.has_value())
                curr_point.decel =
                  units::min(curr_point.decel, *constraint.decel);
        }
    }
}

// computes the isolated constraints
// These constraints do not depend on any other points
void Trajectory::isolatedConstraints() {
    const FLinearAcceleration friction_multiplier =
      m_constraints.coeff_friction * (9.81_Fmps2);

    // half the track width
    const FLength track_radius = m_constraints.track_width * 0.5f;

    for (size_t i = 0; i < getNumPoints(); i++) {
        MotionPoint& point = m_points[i];

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

        // multiplier ensuring no saturation happens
        const float kin_multiplier =
          1.0 / (1.0 + (track_radius * abs_curvature));

        const FLinearVelocity max_kin_vel =
          m_constraints.max_vel * kin_multiplier;
        const FLinearVelocity max_turn_vel =
          m_constraints.max_angular_vel * abs_radius_rad;

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
          m_constraints.max_accel * kin_multiplier;
        const FLinearAcceleration max_turn_accel =
          m_constraints.max_angular_accel * abs_radius_rad;

        const FLinearAcceleration max_kin_decel =
          m_constraints.max_decel * kin_multiplier;
        const FLinearAcceleration max_turn_decel =
          m_constraints.max_angular_decel * abs_radius_rad;

        point.accel = units::min(max_kin_accel, max_turn_accel);
        point.decel = units::min(max_kin_decel, max_turn_decel);
        // point.accel = constraints.max_accel;
        // point.decel = constraints.max_decel;

        if (m_debug_enabled) {
            m_debug_info[i].max_kin_vel = max_kin_vel;
            m_debug_info[i].max_turn_vel = max_turn_vel;
            m_debug_info[i].max_kin_accel = point.accel;
            m_debug_info[i].max_kin_decel = point.decel;
            m_debug_info[i].max_kin_accel = max_kin_accel;
            m_debug_info[i].max_turn_accel = max_turn_accel;
            m_debug_info[i].max_kin_decel = max_kin_decel;
            m_debug_info[i].max_turn_decel = max_turn_decel;
            m_debug_info[i].max_friction_vel = point.vel;
        }
    }

    // apply the extra constraints
    applyPointAndRangeConstraints();

    // sets the start and initial velocity constraints
    // overrides all set constraints
    m_points.front().vel = m_start_vel;
    m_points.back().vel = m_end_vel;
}

FLinearAcceleration Trajectory::get_accel(FLinearVelocity last_vel) {
    const FLength wheel_radius = m_constraints.wheel_diameter / 2.0f;

    const FAngularVelocity last_wheel_ang_vel = rad * (last_vel / wheel_radius);

    const FTorque curr_torque =
      motor_torque2(last_wheel_ang_vel, m_constraints.max_wheel_ang_vel) *
      m_constraints.motor_count;

    // Torque = Force * Radius = (Mass * Accel) * Radius
    // ->
    // Accel = Torque / (Mass * Radius)
    const FLinearAcceleration curr_accel =
      curr_torque / (wheel_radius * m_constraints.robot_mass);

    return curr_accel;
}

// performs a forward pass to keep max acceleration constraints
void Trajectory::forwardsPass() {
    // constant for all points
    const FLength distance_delta_mult = 2 * m_delta_distance;

    // excludes starting point
    for (size_t i = 1; i < m_points.size(); i++) {
        const MotionPoint& last_point = m_points[i - 1];
        MotionPoint& point = m_points[i];
        // (Sprunk 25)

        // predicted max accel from motors
        const FLinearAcceleration motor_accel =
          get_accel(units::abs(last_point.vel));

        const FLinearAcceleration accel =
          units::min(motor_accel, last_point.accel);

        const FLinearVelocity max_vel = units::sqrt(
          units::square(last_point.vel) + accel * distance_delta_mult);

        // keep minimum of current max vel and previous max vel
        point.vel = units::min(point.vel, max_vel);
    }

    // in separate for loop to only execute if statement once
    if (m_debug_enabled) {
        m_debug_info.front().forwards_pass = m_points.front().vel;
        for (size_t i = 1; i < m_points.size(); i++) {
            m_debug_info[i].forwards_pass = m_points[i].vel;
        }
    }
}

// performs a forward pass to keep max deceleration constraints
void Trajectory::backwardsPass() {
    // constant for all points
    const FLength distance_delta_mult = 2 * m_delta_distance;

    // excludes end point
    for (int i = int(getNumPoints()) - 2; i >= 0; i--) {
        // (Sprunk 25)
        MotionPoint& point = m_points[i];
        const MotionPoint& next_point = m_points[i + 1];

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
    }

    // in separate for loop to only execute if statement once
    if (m_debug_enabled) {
        m_debug_info.back().forwards_pass = m_points.back().vel;
        for (int i = int(getNumPoints()) - 2; i >= 0; i--) {
            // is just the same as final velocity?
            m_debug_info[i].backwards_pass = m_points[i].vel;
        }
    }
}

void Trajectory::setTravelTimes() {
    const FLength delta_distance2 = 2 * m_delta_distance;
    m_points[0].travel_time = 0_sec;

    for (size_t i = 1; i < m_points.size(); i++) {
        MotionPoint& point = m_points[i];
        const MotionPoint& last_point = m_points[i - 1];
        // (Sprunk 23)

        // in case vel is negative the delta distance would also be positive, so
        // this term should always be positive
        const Time delta_travel_time =
          units::abs((delta_distance2) / (point.vel + last_point.vel));

        point.travel_time = last_point.travel_time + delta_travel_time;
    }
}

int Trajectory::indexByDistance(FLength distance, int start_ind) const {
    // guess the closest index to the given distance
    int idx_guess = std::round(distance / m_delta_distance);

    const int num_points = getNumPoints();

    // adjust start_ind to avoid lo > hi clamp undefined behavior
    if (start_ind >= num_points) {
        start_ind = num_points - 1;
    }

    return std::clamp(idx_guess, start_ind, num_points - 1);
};

int Trajectory::indexByTime(FTime time, int start_ind) const {
    // actual values of the point don't really matter
    // (except for travel_time)
    MotionPoint query_point = m_points[0];

    query_point.travel_time = time;

    auto travel_time_cmp = [](const MotionPoint& lhs,
                              const MotionPoint& rhs) -> bool {
        return lhs.travel_time < rhs.travel_time;
    };

    // search for time in points
    auto result_itr = lower_bound(m_points.begin(),
                                  m_points.end(),
                                  query_point,
                                  travel_time_cmp);

    int idx = std::distance(m_points.begin(), result_itr);

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
                                    FLength resolution) const {
    FLength best = Length(INFINITY);
    int result = 0;

    FLength original_start_dist = m_points[start_ind].arc_length;
    // either some max look dist or look until the end of the array
    FLength original_end_dist =
      units::min(original_start_dist + max_look_dist, getTotalDistance());

    FLength start_dist = original_start_dist;
    FLength end_dist = original_end_dist;

    for (Length curr_dist = start_dist; curr_dist <= end_dist;
         curr_dist += resolution) {
        // get index by distance
        int i = indexByDistance(curr_dist);

        auto& motion_point = m_points[i];
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

        auto& motion_point = m_points[i];
        Length curr_distance = point.distanceTo(motion_point.point);
        if (curr_distance < best) {
            best = curr_distance;
            result = i;
        }
    }

    // here result is very likely optimal
    return result;
}

size_t Trajectory::sanitizeIndex(int index) const {
    return (size_t)std::clamp(index, 0, int(getNumPoints()) - 1);
}

FDifferentialSpeeds Trajectory::differentialVelocitiesByIndex(int index) const {
    auto& point = getPoint(index);

    return { point.vel, Frad * point.vel * point.curvature };
}

FLength Trajectory::getTotalDistance() const {
    return m_points.back().arc_length;
}

FTime Trajectory::getTotalTime() const {
    return m_points.back().travel_time;
}

size_t Trajectory::getNumPoints() const {
    return m_points.size();
}

const MotionPoint& Trajectory::getPoint(int index) const {
    return m_points.at(index);
}

const MotionPoint& Trajectory::getMotionEndPoint() const {
    return m_points.front();
}

const MotionPoint& Trajectory::getMotionStartPoint() const {
    return m_points.back();
}

Trajectory::Trajectory(std::shared_ptr<geometry::Curve> curve,
                       Constraints constraints,
                       std::vector<PointConstraint> point_constraints,
                       std::vector<RangeConstraint> range_constraints,
                       LinearVelocity start_vel,
                       LinearVelocity end_vel,
                       Length change_in_distance,
                       bool debug_enabled)
    : curve(curve),
      m_constraints(constraints),
      m_start_vel(start_vel),
      m_end_vel(end_vel),
      m_delta_distance(change_in_distance),
      m_point_constraints(point_constraints),
      m_range_constraints(range_constraints),
      m_debug_enabled(debug_enabled) {

    printf("total distance: %f\n", curve->getTotalDistance().convert(in));
    printf("delta_distance: %f\n", m_delta_distance.convert(in));
    size_t estimatedNumPoints =
      static_cast<size_t>(
        (curve->getTotalDistance() / m_delta_distance).internal()) +
      10;

    printf("allocated for points %d\n", estimatedNumPoints);

    // makes the creation of points faster by reserving the required space
    m_points.reserve(estimatedNumPoints);

    if (m_debug_enabled) {
        m_debug_info.reserve(estimatedNumPoints);
    }

    compute();
}
} // namespace mp
} // namespace lyfast
} // namespace blazing
