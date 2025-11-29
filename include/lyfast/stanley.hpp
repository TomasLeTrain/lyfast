#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "lyfast/geometry/curve.hpp"
#include "lyfast/geometry/primitives.hpp"
#include "lyfast/motion_profiling/mp.hpp"
#include "lyfast/vel_controller.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

struct StanleyState {
    bool close;
    std::optional<Time> last_time;
    Time start_time;
    std::optional<Angle> locked_heading;
    size_t last_trajectory_idx;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && linearVelocityTracker<TrackerType> &&
             ArcadeDrivetrain<DrivetrainType> &&
             hasAngularFeedback<ControllersType> &&
             hasLinearFeedback<ControllersType>
class Stanley : public Motion<ControllersType,
                              DrivetrainType,
                              TrackerType,
                              TolerancesType> {
  private:
    std::optional<StanleyState> m_state;
    bool reversed = false;

    Length m_max_lookahead_distance = 6_in;
    Length m_k_curvature = 20_in;
    Divided<Voltage, LinearVelocity> m_k_voltage = 1_volt / 1_mps;

    std::optional<Time> m_timeout = std::nullopt;
    Length close_threshold = 4_in;

    mp::Trajectory* target_trajectory;

  public:
    int getLoopDelayTime() override {
        return 10;
    }

    std::optional<motionExecutionResult> execute() override {
        if (!m_state.has_value()) {
            m_state = { .close = false,
                        .last_time = now(),
                        .start_time = now(),
                        .locked_heading = std::nullopt,
                        .last_trajectory_idx = 0 };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        StanleyState& state = m_state.value();
        motionExecutionResult result;

        Time delta_time = deltaTime(state.last_time);

        const units::V2Position position = this->tracker.getPosition();
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        std::optional<units::V2Position> last_point;
        std::optional<Length> last_error;
        size_t trajectory_idx = state.last_trajectory_idx;

        for (; trajectory_idx < target_trajectory->points.size();
             trajectory_idx++) {
            auto curr_point = target_trajectory->points[trajectory_idx].point;
            auto curr_error = curr_point.distanceTo(position);

            // last point had lower error
            if (last_error.has_value() && curr_error > *last_error) {
                trajectory_idx--;
                break;
            }
            last_point = curr_point;
            last_error = curr_error;
        }

        // no points better than last
        if (trajectory_idx == target_trajectory->points.size()) {
            trajectory_idx--;
        }

        state.last_trajectory_idx = trajectory_idx;

        mp::MotionPoint motion_point =
          target_trajectory->points[trajectory_idx];
        units::V2Position curve_target = motion_point.point;
        Angle curve_angle = motion_point.heading;
        Curvature curve_abs_curvature = units::abs(motion_point.curvature);
        LinearVelocity curve_velocity = motion_point.vel;

        auto local_target_error =
          (position - curve_target).rotatedBy(-curve_angle);
        Length crosstrack_error = local_target_error.y;
        Angle angle_error = angleError(curve_angle, heading);

        // lookahead gets shortened as curvature increases to avoid overshoot on
        // tight turns
        Length lookahead_distance =
          m_max_lookahead_distance / (1 + curve_abs_curvature * m_k_curvature);

        Angle target_steering =
          angle_error + units::atan(crosstrack_error / lookahead_distance);

        auto curve_endpoint = target_trajectory->points.back().point;
        auto distance_to_end = curve_endpoint.distanceTo(position);

        if (units::abs(distance_to_end) < close_threshold && !state.close) {
            // locks heading when close
            auto curve_endpoint_heading =
              target_trajectory->points.back().heading;
            state.locked_heading = curve_endpoint_heading;
            state.close = true;
        }

        // switches to locked heading when close
        Angle target_heading =
          state.locked_heading.value_or(heading + target_steering);

        Angle angular_error = angleError(target_heading, heading);

        // update tolerances
        this->tolerances.linearErrorToleranceUpdate(distance_to_end);
        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker.getLinearVelocity());
        this->tolerances.linearHalfcircleToleranceUpdate(position,
                                                         curve_endpoint,
                                                         heading);

        result.finished = false;

        // check tolerances
        if constexpr (hasLinearTolerance<TolerancesType>) {
            result.inSmallTolerance = this->tolerances.linear.withinTolerance();
            result.finished |= this->tolerances.linear.finished();
        }
        if constexpr (hasLargeLinearTolerance<TolerancesType>) {
            result.inLargeTolerance =
              this->tolerances.large_linear.withinTolerance();
            result.finished |= this->tolerances.large_linear.finished();
        }
        // dont use to check if we have finished
        if constexpr (hasChainLinearTolerance<TolerancesType>) {
            result.inChainTolerance =
              this->tolerances.chain_linear.withinTolerance();
        }

        // check timeout
        result.finished |= timeoutDone(m_timeout, state.start_time);

        // finished if any of the available tolerances or timeout are
        // triggered
        if (result.finished) {
            this->drivetrain.moveArcade(0_volt, 0_volt);
            // returns immediately to avoid more movement
            return result;
        }

        // calculate angular output
        Voltage angular_output =
          this->controllers.angular_feedback.update(-angular_error,
                                                    0_stRad,
                                                    delta_time);

        Voltage linear_output;

        // idea here is to use velocities from motion profile to move most of
        // the way, then use pid for settling
        // as a hack to avoid using velocity controllers (for now) we directly
        // translate velocities to voltages with k_voltage
        linear_output = curve_velocity * m_k_voltage;

        // used only when settling
        if (state.close) {
            linear_output =
              this->controllers.linear_feedback.update(-distance_to_end,
                                                       0.0_in,
                                                       delta_time);
        }

        // constraints should already be inherent to the motion profile

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    Stanley(ControllersType controllers,
            Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
            mp::Trajectory* target_trajectory)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target_trajectory(target_trajectory) {}

    Stanley& getReference() {
        return *this;
    }

    // changer methods

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto reverse() {
        this->reversed = true;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto max_lookahead_distance(Length max_lookahead_distance) {
        this->m_max_lookahead_distance = max_lookahead_distance;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto k_curvature(Length k_curvature) {
        this->m_k_curvature = k_curvature;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto k_voltage(Divided<Voltage, LinearVelocity> k_voltage) {
        this->m_k_voltage = k_voltage;

        return this->getReference();
    }
};

} // namespace lyfast
} // namespace blazing
