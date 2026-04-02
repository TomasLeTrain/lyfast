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
               VelocityArcadeDrivetrain<DrivetrainType> &&
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
    Length close_threshold = 4_in;

    PathFollowParameterizationType m_parameterization_type =
      closest_point_based;
    // look ahead one iteration at a time
    std::variant<Length, Time> m_lookahead = 20_msec;

  public:
    int getLoopDelayTime() override {
        // 20 to allow accurate positioning
        return 20;
    }

    std::optional<motionExecutionResult> execute() override {
        if (!m_state.has_value()) {
            m_state = {
                .last_time = now(),
                .start_time = now(),
                .start_distance = this->tracker.getForwardTravel(),
            };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        PathFollowState& state = m_state.value();
        motionExecutionResult result;

        const Time delta_time = deltaTime(state.last_time);
        const Time elapsed_motion_time = now() - state.start_time;

        const units::V2Position position = this->tracker.getPosition();
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        int reference_idx = [&] {
            if (m_parameterization_type == time_based) {
                // time based
                return target_trajectory->indexByTime(elapsed_motion_time);
            } else if (m_parameterization_type == distance_based) {
                // distance based
                return target_trajectory->indexByDistance(units::max(
                  0_in,
                  this->tracker.getForwardTravel() - state.start_distance));
            } else if (m_parameterization_type == closest_point_based) {
                // closest point based
                // TODO: add max lookahead dist to avoid skipping whole path
                return target_trajectory->indexByClosestPoint(position);
            } else {
                // no parameterization method?
                return -1;
            }
        }();

        // next direct point after current reference. used as fallback on
        // certain cases relating to lookahead
        int fixed_next_reference =
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

        units::Pose reference_pose = { reference_motion_point.point,
                                       reference_motion_point.heading };

        // used as the current state for the feedback controller
        DifferentialSpeeds reference_speeds = {
            reference_motion_point.vel,
            Frad * reference_motion_point.vel * reference_motion_point.curvature
        };

        const mp::MotionPoint& lookahead_motion_point =
          target_trajectory->getPoint(next_reference_idx);

        // units::Pose lookahead_reference_pose = {
        //     lookahead_motion_point.point,
        //     lookahead_motion_point.heading
        // };

        // used as feedforward velocities
        DifferentialSpeeds lookahead_reference_speeds = {
            lookahead_motion_point.vel,
            Frad * lookahead_motion_point.vel * lookahead_motion_point.curvature
        };

        // immediately next reference - target for the feedback control
        // int k1_reference_idx = target_trajectory->indexByTime(
        //   target_trajectory->getPoint(reference_idx).travel_time +
        //     from_msec(getLoopDelayTime()),
        //   // limit to minimum fixed_next_reference
        //   fixed_next_reference);

        // const mp::MotionPoint& k1_reference_motion_point =
        //   target_trajectory->getPoint(k1_reference_idx);

        // units::Pose k1_reference_pose = { k1_reference_motion_point.point,
        //                                   k1_reference_motion_point.heading
        //                                   };
        //
        // DifferentialSpeeds k1_reference_speeds = {
        //     k1_reference_motion_point.vel,
        //     Frad * k1_reference_motion_point.vel *
        //       k1_reference_motion_point.curvature
        // };

        double reverse_multiplier = reversed ? -1.0 : 1.0;

        // TODO: add option for custom settling conditions (different
        // control law maybe)

        // reverse linear_velocity if needed
        reference_speeds.linear_velocity *= reverse_multiplier;
        lookahead_reference_speeds.linear_velocity *= reverse_multiplier;

        // points used for tolerances
        const mp::MotionPoint& curve_endpoint =
          target_trajectory->getMotionEndPoint();
        const Angle curve_endpoint_heading = curve_endpoint.heading;
        const Length distance_to_end =
          curve_endpoint.point.distanceTo(position);

        // update tolerances
        this->tolerances.linearErrorToleranceUpdate(distance_to_end);
        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker.getLinearVelocity());
        this->tolerances.linearHalfcircleToleranceUpdate(
          position,
          curve_endpoint.point,
          curve_endpoint_heading);

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
        auto [left_vel, right_vel] = this->drivetrain.getDrivetrainVelocities();
        auto [actual_volt_left, actual_volt_right] =
          this->drivetrain.getDrivetrainVoltages();

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
        this->drivetrain.moveArcade(
          feedforward_velocities.linear_velocity,
          feedforward_velocities.angular_velocity,
          TargetFeedType { .feedforward = true, .feedback = false });

        // update feedback vel
        this->drivetrain.moveArcade(
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

    motionChangerMsg PathFollow& closeThreshold(Length threshold) {
        this->close_threshold = threshold;
        return *this;
    }

    motionChangerMsg PathFollow& timeout(Time timeout) {
        this->m_timeout = timeout;

        return *this;
    }

}; // namespace lyfast

} // namespace lyfast
} // namespace blazing
