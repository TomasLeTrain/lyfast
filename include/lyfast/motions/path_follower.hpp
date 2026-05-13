#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "lyfast/controllers/path_pose_feedback.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "lyfast/motion_profiling/mp.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <memory>
#include <variant>

namespace blazing {
namespace lyfast {
struct PathFollowState {
    std::optional<Time> last_time;
    Time start_time;
    Length start_distance;
    int last_reference_idx;
};

enum PathFollowParameterizationType {
    time_based,
    distance_based,
    closest_point_based
};

// generic motion for path following supporting various feedback control laws
template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && forwardTravelTracker<TrackerType> &&
               VelocityArcadeFeedtypeDrivetrain<DrivetrainType> &&
               hasPathPoseFeedback<ControllersType>
class PathFollow : public Motion<ControllersType,
                                 DrivetrainType,
                                 TrackerType,
                                 TolerancesType,
                                 PathFollow<ControllersType,
                                            DrivetrainType,
                                            TrackerType,
                                            TolerancesType>>,
                   public LinearMotion<PathFollow<ControllersType,
                                                  DrivetrainType,
                                                  TrackerType,
                                                  TolerancesType>>,
                   public AngularMotion<PathFollow<ControllersType,
                                                   DrivetrainType,
                                                   TrackerType,
                                                   TolerancesType>> {
  private:
    std::optional<PathFollowState> m_state;
    bool reversed = false;

    std::shared_ptr<mp::Trajectory> target_trajectory;

    std::optional<Time> m_timeout = std::nullopt;

    PathFollowParameterizationType m_parameterization_type =
      closest_point_based;
    // look ahead one iteration at a time
    std::variant<Length, Time> m_lookahead = 20_msec;

    // maximum distance the lookahead point can deviate from the previous
    // done to prevent skipping entire sections of the path in case of
    // intersections
    Length m_max_lookahead_dist = 10_in;

  public:
    int getLoopDelayTime() override {
        // 20 to allow accurate positioning
        return 20;
    }

    std::optional<motionExecutionResult> execute() override {
        if (!m_state.has_value()) {
            m_state = { .last_time = now(),
                        .start_time = now(),
                        .start_distance = this->tracker->getForwardTravel(),
                        .last_reference_idx = 0 };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        PathFollowState& state = m_state.value();
        motionExecutionResult result;

        const Time delta_time = deltaTime(state.last_time);
        const Time elapsed_motion_time = now() - state.start_time;

        double reverse_multiplier = reversed ? -1.0 : 1.0;

        // reverses heading if neccesary
        auto applyHeadingReversal = [&](Angle angle) -> Angle {
            return reversed ? reverseAngle(angle) : angle;
        };

        const units::V2Position position = this->tracker->getPosition();
        const Angle heading = this->tracker->getAngle();

        int reference_idx = [&] {
            if (m_parameterization_type == time_based) {
                // time based
                return target_trajectory->indexByTime(elapsed_motion_time);
            } else if (m_parameterization_type == distance_based) {
                // distance based
                const Length distance_traveled = units::max(
                  0_in,
                  reverse_multiplier *
                    (this->tracker->getForwardTravel() - state.start_distance));
                return target_trajectory->indexByDistance(distance_traveled);
            } else if (m_parameterization_type == closest_point_based) {
                // closest point based
                return target_trajectory->indexByClosestPoint(
                  position,
                  state.last_reference_idx,
                  m_max_lookahead_dist);
            } else {
                // no parameterization method?
                return -1;
            }
        }();

        state.last_reference_idx = reference_idx;

        // next direct point after current reference. used as fallback on
        // certain cases relating to lookahead
        const int fixed_next_reference =
          target_trajectory->sanitizeIndex(reference_idx + 1);

        // in case there is no lookahead we use the next index point
        int next_reference_idx = fixed_next_reference;

        // performs the lookahead logic
        if (std::holds_alternative<Length>(m_lookahead)) {
            next_reference_idx = target_trajectory->indexByDistance(
              target_trajectory->getPoint(reference_idx).arc_length +
                std::get<Length>(m_lookahead),
              // limit to minimum fixed_next_reference
              fixed_next_reference);
        } else if (std::holds_alternative<Time>(m_lookahead)) {
            next_reference_idx = target_trajectory->indexByTime(
              target_trajectory->getPoint(reference_idx).travel_time +
                std::get<Time>(m_lookahead),
              // limit to minimum fixed_next_reference
              fixed_next_reference);
        }

        const mp::MotionPoint& reference_motion_point =
          target_trajectory->getPoint(reference_idx);
        const mp::MotionPoint& lookahead_motion_point =
          target_trajectory->getPoint(next_reference_idx);

        units::Pose reference_pose = reference_motion_point.pose();
        // changes heading to match backwards movement
        reference_pose.orientation =
          applyHeadingReversal(reference_pose.orientation);

        // used as the current state for the feedback controller
        DifferentialSpeeds reference_speeds =
          reference_motion_point.calculateSpeeds();

        // used as feedforward velocities
        DifferentialSpeeds lookahead_reference_speeds =
          lookahead_motion_point.calculateSpeeds();

        // reverse linear vels if needed
        reference_speeds.linear_velocity *= reverse_multiplier;
        lookahead_reference_speeds.linear_velocity *= reverse_multiplier;

        // TODO: add option for custom settling conditions (different
        // control law maybe)

        // update tolerances
        {
            const auto distance_to_end = target_trajectory->getTotalDistance() -
                                         reference_motion_point.arc_length;
            this->tolerances.linearErrorToleranceUpdate(distance_to_end);
        }
        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker->getLinearVelocity());
        {
            const auto curve_endpoint_pose =
              target_trajectory->getMotionEndPoint().pose();

            this->tolerances.linearHalfcircleToleranceUpdate(
              position,
              curve_endpoint_pose,
              curve_endpoint_pose.orientation);
        }

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

            this->tolerances.chain_linear.finished();
        }

        // check timeout
        result.finished |= timeoutDone(m_timeout, state.start_time);

        // finished if any of the available tolerances or timeout are
        // triggered
        if (result.finished) {
            this->drivetrain->moveArcade(0_volt, 0_volt);
            // returns immediately to avoid more movement
            return result;
        }

        // current state
        PathPoseFeedbackT path_pose_state {
            .pose = units::Pose { position, heading },
            // use from current reference
            .velocities = reference_speeds
        };

        // seeking the next reference
        PathPoseFeedbackT path_pose_reference {
            // .pose = k1_reference_pose,
            .pose = reference_pose,
            // target next reference
            // .velocities = k1_reference_speeds
            .velocities = reference_speeds
        };

        // get purely feedback term of the feedback control velocities
        DifferentialSpeeds lqr_feedback_velocities =
          this->controllers.path_pose_feedback.update(path_pose_state,
                                                      path_pose_reference,
                                                      delta_time) -
          path_pose_reference.velocities;

        // lookahead reference vel
        DifferentialSpeeds feedforward_velocities =
          lookahead_reference_speeds + lqr_feedback_velocities;

        // current reference + lqr feedback
        DifferentialSpeeds feedback_velocities =
          reference_speeds + lqr_feedback_velocities;

        // linear and angular should be references for the velocity
        // controller
        auto [left_vel, right_vel] =
          this->drivetrain->getDrivetrainVelocities();
        auto [actual_volt_left, actual_volt_right] =
          this->drivetrain->getDrivetrainVoltages();

        std::cout << std::fixed;
        std::cout << std::setprecision(5);

        std::cout << "lin/ang/drive_left/drive_right/tv_l/tv_r/"
                     "av_l/av_r/x/y/theta/tx/ty/ttheta: "
                  << feedforward_velocities.linear_velocity.internal() << " "
                  << feedforward_velocities.angular_velocity.internal() << " "
                  << left_vel.internal() << " " << right_vel.internal()
                  << " "
                  // << saturated_voltages.at(0).internal() << " "
                  // << saturated_voltages.at(1).internal() << " "
                  // << 0 << " " << 0 << " "
                  // << actual_volt_left.internal() << " "
                  // << actual_volt_right.internal() << " "
                  << reference_speeds.linear_velocity.internal() << " "
                  << reference_speeds.angular_velocity.internal() << " "
                  << actual_volt_left.internal() << " "
                  << actual_volt_right.internal() << " "
                  << position.x.convert(in) << " " << position.y.convert(in)
                  << " " << heading.convert(deg) << " "
                  << path_pose_reference.pose.x.convert(in) << " "
                  << path_pose_reference.pose.y.convert(in) << " "
                  << path_pose_reference.pose.orientation.convert(deg)
                  << std::endl;

        // update feedforward vel
        this->drivetrain->moveArcade(
          feedforward_velocities.linear_velocity,
          feedforward_velocities.angular_velocity,
          TargetFeedType { .feedforward = true, .feedback = false });

        // update feedback vel
        this->drivetrain->moveArcade(
          feedback_velocities.linear_velocity,
          feedback_velocities.angular_velocity,
          TargetFeedType { .feedforward = false, .feedback = true });

        return result;
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    PathFollow(ControllersType controllers,
               Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
               std::shared_ptr<mp::Trajectory> target_trajectory)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 PathFollow<ControllersType,
                            DrivetrainType,
                            TrackerType,
                            TolerancesType>>(controllers, chassis),
          target_trajectory(target_trajectory) {}

    // changer methods
    motionChangerMsg PathFollow& reverse() {
        this->reversed = true;

        return *this;
    }

    motionChangerMsg PathFollow& setReverse(bool reversed) {
        this->reversed = reversed;

        return *this;
    }

    motionChangerMsg PathFollow&
    lookahead(std::variant<Length, Time> lookahead) {
        this->m_lookahead = lookahead;
        return *this;
    }

    motionChangerMsg PathFollow&
    parameterization(PathFollowParameterizationType parameterization_type) {
        this->m_parameterization_type = parameterization_type;
        return *this;
    }

    motionChangerMsg PathFollow& timeout(Time timeout) {
        this->m_timeout = timeout;

        return *this;
    }

}; // namespace lyfast

} // namespace lyfast
} // namespace blazing
