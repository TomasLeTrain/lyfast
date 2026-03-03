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
    std::variant<float, FLength> timeframe;

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

    std::shared_ptr<geometry::Curve> curve;

  private:
    std::shared_ptr<geometry::Curve> getCurve();

    void compute();

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
    Constraints constraints;
    FLinearVelocity start_vel, end_vel;

    std::vector<MotionPoint> points;

    // change in distance between points
    FLength delta_distance;

    std::vector<PointConstraint> point_constraints;

    int indexByDistance(FLength distance, int start_ind = 0);
    int indexByTime(FTime time, int start_ind = 0);

    int indexByClosestPoint(geometry::Point point,
                            int start_ind = 0,
                            FLength max_dist = Length(INFINITY),
                            FLength resolution = 1_in);

    FLength getTotalDistance();
    FTime getTotalTime();

    size_t getNumPoints();
    MotionPoint& getPoint(int index);
    int sanitizeIndex(int index);

    MotionPoint& getMotionEndPoint();
    MotionPoint& getMotionStartPoint();

    FDifferentialSpeeds differentialVelocitiesByIndex(int index);

    Trajectory(std::shared_ptr<geometry::Curve> curve,
               Constraints constraints,
               std::vector<PointConstraint> point_constraints,
               LinearVelocity start_vel,
               LinearVelocity end_vel,
               Length change_in_distance);
};
} // namespace mp
} // namespace lyfast
} // namespace blazing
