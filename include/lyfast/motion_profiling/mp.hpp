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

    FDifferentialSpeeds calculateSpeeds() const {
        return { vel, unit_cast<AngularVelocity>(vel * curvature) };
    }

    units::FPose pose() const {
        return { point, heading };
    }
};

class Trajectory {
  public:
    struct debugInfo {
        FLinearVelocity max_kin_vel;
        FLinearVelocity max_turn_vel;
        FLinearVelocity max_friction_vel;

        FLinearVelocity forwards_pass;
        FLinearVelocity backwards_pass;

        FLinearAcceleration max_kin_accel;
        FLinearAcceleration max_turn_accel;
        FLinearAcceleration max_kin_decel;
        FLinearAcceleration max_turn_decel;

        FLinearVelocity final_vels;
    };

  private:
    std::vector<debugInfo> m_debug_info;

    std::shared_ptr<geometry::Curve> curve;
    Constraints m_constraints;
    FLinearVelocity m_start_vel, m_end_vel;

    std::vector<MotionPoint> m_points;

    // change in distance between points
    FLength m_delta_distance;

    std::vector<PointConstraint> m_point_constraints;
    std::vector<RangeConstraint> m_range_constraints;
    bool m_debug_enabled;

  private:
    // computes the path and motion profile
    void compute();

    // finds an index given a constraint keyframe (spline time or length)
    size_t indexByKeyframe(std::variant<float, FLength>& keyframe);

    // applies extra given point/range constraints
    void applyPointAndRangeConstraints();

    // computes the isolated constraints
    // These constraints do not depend on any other points
    void isolatedConstraints();

    FLinearAcceleration get_accel(FLinearVelocity last_vel);

    // performs a forward pass to keep max acceleration constraints
    void forwardsPass();

    // performs a forward pass to keep max deceleration constraints
    void backwardsPass();

    // computes travel time for all the points
    void setTravelTimes();

  public:
    // returns the curve the motion profile is using
    std::shared_ptr<geometry::Curve> getCurve();

    Constraints getConstraints() const;
    FLength getDeltaDistance() const;

    const std::vector<debugInfo>& getDebugInfo() const;
    const std::vector<MotionPoint>& getPoints() const;

    const std::vector<PointConstraint>& getPointConstraints() const;
    const std::vector<RangeConstraint>& getRangeConstraints() const;
    bool getDebugEnabled() const;

    FLinearVelocity getStartVelocity() const;
    FLinearVelocity getEndVelocity() const;

    // returns the index of the point with a given distance, rounded to the
    // nearest index
    int indexByDistance(FLength distance, int start_ind = 0) const;

    // returns the index of the point with a given time, rounded to the
    // nearest index
    int indexByTime(FTime time, int start_ind = 0) const;

    // returns the index of the point on the curve closest to the given point.
    // The point found has an index greater than start_ind and at most a
    // max_dist distance from start_ind. resolution is used to search for the
    // point in the given max_dist window, reducing increases lookup time and
    // may no neccesarily increase precision (can get to within index precision)
    int indexByClosestPoint(geometry::Point point,
                            int start_ind = 0,
                            FLength max_dist = Length(INFINITY),
                            FLength resolution = 1_in) const;

    // return total arc length of the path
    FLength getTotalDistance() const;

    // return total calculated travel time of the path
    FTime getTotalTime() const;

    // returns total number of points
    size_t getNumPoints() const;

    // returns const reference to point at specified index
    const MotionPoint& getPoint(int index) const;

    // sanitizes a given index to be within a valid range
    size_t sanitizeIndex(int index) const;

    // returns first motion point
    const MotionPoint& getMotionStartPoint() const;

    // returns last motion point
    const MotionPoint& getMotionEndPoint() const;

    // returns the target linear/angular velocities at some index
    FDifferentialSpeeds differentialVelocitiesByIndex(int index) const;

    Trajectory(std::shared_ptr<geometry::Curve> curve,
               Constraints constraints,
               std::vector<PointConstraint> point_constraints,
               std::vector<RangeConstraint> range_constraints,
               LinearVelocity start_vel,
               LinearVelocity end_vel,
               Length change_in_distance,
               bool debug_enabled = false);
};
} // namespace mp
} // namespace lyfast
} // namespace blazing
