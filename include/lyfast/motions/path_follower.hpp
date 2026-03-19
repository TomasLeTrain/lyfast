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
              // limit to fixed_next_reference
              fixed_next_reference);
        } else if (std::holds_alternative<Time>(m_lookahead)) {
            next_reference_idx = target_trajectory->indexByTime(
              target_trajectory->getPoint(reference_idx).travel_time +
                std::get<Time>(m_lookahead),
              // limit to fixed_next_reference
              fixed_next_reference);
        }

        const mp::MotionPoint& reference_motion_point =
          target_trajectory->getPoint(reference_idx);

        // units::Pose reference_pose = { reference_motion_point.point,
        //                                reference_motion_point.heading };

        DifferentialSpeeds reference_speeds = {
            reference_motion_point.vel,
            Frad * reference_motion_point.vel * reference_motion_point.curvature
        };

        const mp::MotionPoint& next_reference_motion_point =
          target_trajectory->getPoint(next_reference_idx);

        units::Pose next_reference_pose = {
            next_reference_motion_point.point,
            next_reference_motion_point.heading
        };

        DifferentialSpeeds next_reference_speeds = {
            next_reference_motion_point.vel,
            Frad * next_reference_motion_point.vel *
              next_reference_motion_point.curvature
        };

        double reverse_multiplier = reversed ? -1.0 : 1.0;

        // TODO: add option for custom settling conditions (different
        // control law maybe)

        // reverse linear_velocity if needed
        reference_speeds.linear_velocity *= reverse_multiplier;
        next_reference_speeds.linear_velocity *= reverse_multiplier;

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
            // TODO: use actual speeds?
            .velocities = reference_speeds
        };

        // seeking the next reference
        PathPoseFeedbackT path_pose_reference { .pose = next_reference_pose,
                                                // target next reference
                                                .velocities =
                                                  next_reference_speeds };

        // use feedback control law to figure out new velocities
        DifferentialSpeeds new_speeds =
          this->controllers.path_pose_feedback.update(path_pose_state,
                                                      path_pose_reference,
                                                      delta_time);

        // linear and angular should be references for the velocity
        // controller

        auto [left_vel, right_vel] = this->drivetrain.getDrivetrainVelocities();
        auto [actual_volt_left, actual_volt_right] =
          this->drivetrain.getDrivetrainVoltages();

        std::cout << std::fixed;
        std::cout << std::setprecision(5);

        std::cout << "lin/ang/drive_left/drive_right/tv_l/tv_r/"
                     "av_l/av_r/x/y/theta/tx/ty/ttheta: "
                  << new_speeds.linear_velocity.internal() << " "
                  << new_speeds.angular_velocity.internal() << " "
                  << left_vel.internal() << " " << right_vel.internal()
                  << " "
                  // << saturated_voltages.at(0).internal() << " "
                  // << saturated_voltages.at(1).internal() << " "
                  << 0 << " " << 0 << " " << actual_volt_left.internal() << " "
                  << actual_volt_right.internal() << " "
                  << position.x.convert(in) << " " << position.y.convert(in)
                  << " " << heading.convert(deg) << " "
                  << path_pose_reference.pose.x.convert(in) << " "
                  << path_pose_reference.pose.y.convert(in) << " "
                  << path_pose_reference.pose.orientation.convert(deg)
                  << std::endl;

        this->drivetrain.moveArcade(new_speeds.linear_velocity,
                                    new_speeds.angular_velocity);

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
