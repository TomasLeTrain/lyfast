#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "liblvgl/misc/lv_types.h"
#include "lyfast/motion_profiling/mp.hpp"
#include "lyfast/vel_controller.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <variant>

namespace blazing {
namespace lyfast {

struct PathFollowState {
    std::optional<Time> last_time;
    Time start_time;
    Length start_distance;
};

// generic motion for path following supporting various feedback control laws
template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && forwardTravelTracker<TrackerType> &&
               TankDrivetrain<DrivetrainType> &&
               hasVelocityFeedforward<ControllersType>
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

    // TODO: pointer??
    mp::Trajectory* target_trajectory;

    std::optional<Time> m_timeout = std::nullopt;
    Length close_threshold = 4_in;

    enum parameterizationType {
        time_based,
        distance_based,
        closest_point_based
    };

    parameterizationType m_parameterization_type;
    std::variant<Length, Time> m_lookahead = 0_in;

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

        Time delta_time = deltaTime(state.last_time);
        const Time elapsed_motion_time = now() - state.start_time;

        const units::V2Position position = this->tracker.getPosition();
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            // TODO: maybe reversing should be part of the motion?
            return reversed ? reverseAngle(heading) : heading;
        }();

        int target_idx = -1;

        if (m_parameterization_type == time_based) {
            // time based
            target_idx =
              target_trajectory->indexByTime(elapsed_motion_time);
        } else if (m_parameterization_type == distance_based) {
            // distance based
            target_idx = target_trajectory->indexByDistance(units::max(
              0_in,
              this->tracker.getForwardTravel() - state.start_distance));
        } else if (m_parameterization_type == closest_point_based) {
            // closest point based
            target_idx = target_trajectory->indexByClosestPoint(position);
        }

        // performs the lookahead logic
        // TODO: possibly could make more performant?
        if (std::holds_alternative<Length>(m_lookahead)) {
            target_idx = target_trajectory->indexByDistance(
              target_trajectory->points[target_idx].arc_length +
              std::get<Length>(m_lookahead));
        } else {
            target_idx = target_trajectory->indexByTime(
              target_trajectory->points[target_idx].travel_time +
              std::get<Time>(m_lookahead));
        }

        mp::MotionPoint target_motion_point =
          target_trajectory->points[target_idx];

        units::Pose target = { target_motion_point.point,
                               target_motion_point.heading };

        DifferentialSpeeds target_speeds = { target_motion_point.vel,
                                             Frad * target_motion_point.vel *
                                               target_motion_point.curvature };

        units::V2Position local_error = (target - position).rotatedBy(-heading);

        double reverse_multiplier = reversed ? -1.0 : 1.0;

        // reverse linear_velocity if needed
        target_speeds.linear_velocity *= reverse_multiplier;
        // reverse angular as well?

        Angle errorAngle = angleError(target.orientation, heading);

        DifferentialSpeeds new_speeds;

        // v_new = cos(e_theta) * v + k * e_x
        new_speeds.linear_velocity =
          units::cos(errorAngle) * target_speeds.linear_velocity +
          k * local_error.x;

        // w_new = w + k * e_theta + beta * v * sinc(e_theta) * e_y
        new_speeds.angular_velocity = target_speeds.angular_velocity +
                                      k * errorAngle +
                                      beta * target_speeds.linear_velocity *
                                        sinc(errorAngle) * local_error.y;

        LeftRightVoltages voltages =
          this->controllers.velocity_feedforward.update(new_speeds, delta_time);

        auto curve_endpoint = target_trajectory->points.back().point;
        auto curve_endpoint_heading = target_trajectory->points.back().heading;
        auto distance_to_end = curve_endpoint.distanceTo(position);

        // update tolerances
        this->tolerances.linearErrorToleranceUpdate(distance_to_end);
        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker.getLinearVelocity());
        this->tolerances.linearHalfcircleToleranceUpdate(
          position,
          curve_endpoint,
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

        std::array<Voltage, 2> saturated_voltages { voltages.left_voltage,
                                                    voltages.right_voltage };

        // normalizes voltages to [-1, 1]
        auto [normal_left_voltage, normal_right_voltage] =
          desaturate(saturated_voltages, 1_volt);

        // std::cout << std::format("pos: {:.2f} {:.2f}, error: {:.2f} "
        //                          "{:.2f}, target: {:.2f} {:.2f} k {:.4f}",
        //                          // "lin/alg: {:.2f} {:.2f}, k {:.4f}",
        //                          // new_speeds.linear_velocity.convert(inps),
        //                          //
        //                          new_speeds.angular_velocity.convert(radps),
        //                          // k.internal())
        //                          position.x.convert(in),
        //                          position.y.convert(in),
        //                          local_error.x.convert(in),
        //                          local_error.y.convert(in),
        //                          target.x.convert(in),
        //                          target.y.convert(in),
        //                          k.internal())
        //           << std::endl;

        this->drivetrain.moveTank(normal_left_voltage, normal_right_voltage);

        return result;
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    PathFollow(ControllersType controllers,
               Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
               mp::Trajectory* target_trajectory,
               zeta_units zeta,
               beta_units beta)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 PathFollow<ControllersType,
                            DrivetrainType,
                            TrackerType,
                            TolerancesType>>(controllers, chassis),
          target_trajectory(target_trajectory),
          zeta(zeta),
          beta(beta) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    PathFollow(ControllersType controllers,
               Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
               mp::Trajectory* target_trajectory,
               double zeta,
               double beta)
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
    lookahead(parameterizationType parameterization_type) {
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
