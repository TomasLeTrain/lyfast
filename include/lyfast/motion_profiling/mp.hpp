#pragma once

#include "blazing/utils.hpp"
#include "lyfast/geometry/curve.hpp"
#include "lyfast/motion_profiling/constraints.hpp"
#include "lyfast/utils/motor_dynamics.hpp"
#include "pros/rtos.h"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace blazing {
namespace lyfast {
namespace mp {

struct PointConstraint {
    // spline time or length based
    std::variant<float, FLength> keyframe;

    std::optional<FLinearVelocity> vel = std::nullopt;
    std::optional<FLinearAcceleration> accel = std::nullopt;
    std::optional<FLinearAcceleration> decel = std::nullopt;
};

struct RangeConstraint {
    // spline time or length based
    std::variant<float, FLength> left_keyframe;
    std::variant<float, FLength> right_keyframe;

    std::optional<FLinearVelocity> vel = std::nullopt;
    std::optional<FLinearAcceleration> accel = std::nullopt;
    std::optional<FLinearAcceleration> decel = std::nullopt;
};

struct MotionPoint {
    geometry::Point point;
    FCurvature curvature;
    FAngle heading;
    FLength arc_length;

    FLinearVelocity vel = LinearVelocity(infinity());

    FLinearAcceleration accel = LinearAcceleration(infinity());
    FLinearAcceleration decel = LinearAcceleration(infinity());

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
    // all for debugging:
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
    // debug end

    std::shared_ptr<geometry::Curve> curve;

  private:
    void compute();

    // finds an index given a constraint keyframe (spline time or length)
    size_t indexByKeyframe(std::variant<float, FLength>& keyframe);

    // applies extra given constraints
    void applyPointAndRangeConstraints();

    // computes the isolated constraints
    // These constraints do not depend on any other points
    void isolatedConstraints();

    FLinearAcceleration get_accel(FLinearVelocity last_vel);

    // performs a forward pass to keep max acceleration constraints
    void forwardsPass();

    // performs a forward pass to keep max deceleration constraints
    void backwardsPass();

    void setTravelTimes();

  public:
    Constraints m_constraints;
    FLinearVelocity m_start_vel, m_end_vel;

    std::vector<MotionPoint> m_points;

    // change in distance between points
    FLength m_delta_distance;

    std::vector<PointConstraint> m_point_constraints;
    std::vector<RangeConstraint> m_range_constraints;

    // returns the curve the motion profile is using
    std::shared_ptr<geometry::Curve> getCurve();

    // returns the index of the point with a given distance, rounded to the
    // nearest index
    int indexByDistance(FLength distance, int start_ind = 0);

    // returns the index of the point with a given time, rounded to the
    // nearest index
    int indexByTime(FTime time, int start_ind = 0);

    // returns the index of the point on the curve closest to the given point.
    // The point found has an index greater than start_ind and at most a
    // max_dist distance from start_ind. resolution is used to search for the
    // point in the given max_dist window, reducing increases lookup time and
    // may no neccesarily increase precision (can get to within index precision)
    int indexByClosestPoint(geometry::Point point,
                            int start_ind = 0,
                            FLength max_dist = Length(INFINITY),
                            FLength resolution = 1_in);

    FLength getTotalDistance();
    FTime getTotalTime();

    size_t getNumPoints();
    MotionPoint& getPoint(int index);

    // sanitizes a given index to be within a valid range
    size_t sanitizeIndex(int index);

    MotionPoint& getMotionEndPoint();
    MotionPoint& getMotionStartPoint();

    FDifferentialSpeeds differentialVelocitiesByIndex(int index);

    Trajectory(std::shared_ptr<geometry::Curve> curve,
               Constraints constraints,
               std::vector<PointConstraint> point_constraints,
               std::vector<RangeConstraint> range_constraints,
               LinearVelocity start_vel,
               LinearVelocity end_vel,
               Length change_in_distance);
};
} // namespace mp
} // namespace lyfast
} // namespace blazing
