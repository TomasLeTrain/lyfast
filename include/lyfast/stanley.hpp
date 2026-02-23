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
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && linearVelocityTracker<TrackerType> &&
               ArcadeDrivetrain<DrivetrainType> &&
               hasAngularFeedback<ControllersType> &&
               hasLinearFeedback<ControllersType> &&
               hasVelocityFeedforward<ControllersType>
class Stanley
    : public Motion<
        ControllersType,
        DrivetrainType,
        TrackerType,
        TolerancesType,
        Stanley<ControllersType, DrivetrainType, TrackerType, TolerancesType>>,
      public LinearMotion<
        Stanley<ControllersType, DrivetrainType, TrackerType, TolerancesType>>,
      public AngularMotion<
        Stanley<ControllersType, DrivetrainType, TrackerType, TolerancesType>> {

  private:
    std::optional<StanleyState> m_state;
    bool reversed = false;

    // Length m_max_lookahead_distance = 6_in;
    // Length m_k_curvature = 20_in;
    Divided<Number, Time> m_k = 1 / sec;

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
                        .locked_heading = std::nullopt };
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

        size_t trajectory_idx =
          target_trajectory->indexByClosestPoint(position);

        mp::MotionPoint motion_point =
          target_trajectory->points[trajectory_idx];
        units::V2Position curve_target = motion_point.point;
        Angle curve_angle = motion_point.heading;
        // Curvature curve_abs_curvature = units::abs(motion_point.curvature);
        LinearVelocity curve_velocity = motion_point.vel;

        auto local_target_error =
          (position - curve_target).rotatedBy(-curve_angle);
        Length crosstrack_error = local_target_error.y;
        Angle angle_error = angleError(curve_angle, heading);

        Angle target_steering =
          angle_error + units::atan(m_k * crosstrack_error / curve_velocity);

        auto curve_endpoint = target_trajectory->points.back().point;
        auto curve_endpoint_heading = target_trajectory->points.back().heading;
        auto distance_to_end = curve_endpoint.distanceTo(position);

        double distance_sign =
          signed_sgn(units::cos(angleError(curve_endpoint_heading, heading)));

        if (units::abs(distance_to_end) < close_threshold && !state.close) {
            // locks heading when close
            std::cout << "locked heading as "
                      << curve_endpoint_heading.convert(deg) << std::endl;
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

        // used only for linear output (?)
        DifferentialSpeeds new_speeds = { curve_velocity, 0_radps };
        LeftRightVoltages voltages =
          this->controllers.velocity_feedforward.update(new_speeds, delta_time);
        linear_output = (voltages.left_voltage + voltages.right_voltage) / 2.0;

        // used only when settling
        if (state.close) {
            std::cout << "using pid: "
                      << -distance_to_end.convert(in) * distance_sign
                      << std::endl;
            linear_output = this->controllers.linear_feedback.update(
              -distance_to_end * distance_sign,
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
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 Stanley<ControllersType,
                         DrivetrainType,
                         TrackerType,
                         TolerancesType>>(controllers, chassis),
          target_trajectory(target_trajectory) {}

    // changer methods

    motionChangerMsg Stanley& reverse() {
        this->reversed = true;

        return *this;
    }

    motionChangerMsg Stanley& k(Divided<Number, Time> k) {
        m_k = k;

        return *this;
    }

    motionChangerMsg Stanley& closeThreshold(Length threshold) {
        this->close_threshold = threshold;
        return *this;
    }

    motionChangerMsg Stanley& timeout(Time timeout) {
        this->m_timeout = timeout;

        return *this;
    }
};

} // namespace lyfast
} // namespace blazing
