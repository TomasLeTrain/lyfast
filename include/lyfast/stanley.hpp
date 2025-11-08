#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "lyfast/geometry/curve.hpp"
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
    float last_t;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && linearVelocityTracker<TrackerType> &&
             TankDrivetrain<DrivetrainType> &&
             hasVelocityFeedforward<ControllersType>
class Stanley : public Motion<ControllersType,
                              DrivetrainType,
                              TrackerType,
                              TolerancesType> {
  private:
    std::optional<StanleyState> m_state;
    bool reversed = false;

    lyfast::geometry::Curve* target_curve;

    const Divided<Number, Time> k = 1 / sec;

  public:
    int getLoopDelayTime() override {
        return 10;
    }

    // moveTo-specific properties
    std::optional<Time> m_timeout = std::nullopt;
    Length close_threshold = 4_in;

    std::optional<motionExecutionResult> execute() override {
        if (!m_state.has_value()) {
            m_state = { .close = false,
                        .last_time = now(),
                        .start_time = now(),
                        .locked_heading = std::nullopt,
                        .last_t = 0.0 };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        StanleyState& state = m_state.value();
        motionExecutionResult result;

        // should never equal 0_sec
        Time delta_time = deltaTime(state.last_time);

        const units::V2Position position = this->tracker.getPosition();
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        // Length linear_error =
        //   (target - position).magnitude() * (reversed ? -1.0 : 1.0);

        float t = state.last_t;
        std::optional<Length> last_error = std::nullopt;
        bool decreasing = false;

        while (true) {
            auto curve_target = target_curve->f(t);
            auto curr_error = curve_target.distanceTo(position);
            if (!last_error) {
                last_error = curr_error;
            } else if (curr_error < *last_error) {
                decreasing = true;
            } else if (curr_error < *last_error) {
            }
        }

        units::V2Position curve_target = target_curve->f(t);
        Angle curve_angle = target_curve->df(t).getAngle();

        auto local_target_error =
          (position - curve_target).rotatedBy(-curve_angle);
        Length crosstrack_error = local_target_error.y;
        Angle theta_difference = heading - curve_angle;

        LinearVelocity velocity = 10_mps;

        Angle target_steering =
          theta_difference + units::atan(k * crosstrack_error / velocity);

        Angle position_target_heading = position.angleTo(target);

        if (units::abs(local_target_error) < close_threshold && !state.close) {
            state.locked_heading = position_target_heading;
            state.close = true;
        }

        // switches to locked heading when close
        Angle target_heading = state.locked_heading ? *state.locked_heading :
                                                      position_target_heading;

        // used to determine sign and cosine scaling of linear output
        Angle position_target_error =
          angleError(position_target_heading, heading);

        Angle angular_error = angleError(target_heading, heading);

        // used for cosine scaling and applying correct sign for linear
        // error/output
        Number lin_multiplier = angular_linear_func(position_target_error);

        // applies sign component here so that sign of error is accurate
        // NOTE: sgn can be zero, which can set linear error to zero as
        // well!
        local_target_error *= signed_sgn(lin_multiplier);

        this->tolerances.linearErrorToleranceUpdate(local_target_error);
        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker.getLinearVelocity());
        this->tolerances.linearHalfcircleToleranceUpdate(position,
                                                         target,
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

        // calculate outputs
        Voltage angular_output =
          this->controllers.angular_feedback.update(-angular_error,
                                                    0_stRad,
                                                    delta_time);

        Voltage linear_output =
          this->controllers.linear_feedback.update(-local_target_error,
                                                   0.0_in,
                                                   delta_time);

        if (m_k_lat) {
            angular_output =
              angular_output + *m_k_lat * linear_output *
                                 (target - position).rotatedBy(-heading).y *
                                 sinc(angular_error);
        }

        // sign was already applied to error, only applies cosine scaling
        // component
        linear_output *= units::abs(lin_multiplier);

        // here the robot would attempt to move backwards, when instead the
        // robot should turn around until it should start moving towards the
        // target
        // the reason that this is done to linear_output and not
        // linear_error is because otherwise linear_error would be zero and
        // tolerances would trigger
        if (!state.close && lin_multiplier < 0) {
            linear_output = 0_volt;
        }

        // apply min voltage constraints
        if constexpr (hasLinearVoltageClamp<ControllersType>) {
            linear_output =
              this->controllers.linear_voltage_clamp.applyMin(linear_output);
        }
        if constexpr (hasAngularVoltageClamp<ControllersType>) {
            angular_output =
              this->controllers.angular_voltage_clamp.applyMin(angular_output);
        }

        if (max_overturn_output) {
            // apply overturn
            Voltage overturn_value = units::abs(linear_output) +
                                     units::abs(angular_output) -
                                     *max_overturn_output;

            if (overturn_value > 0_volt) {
                linear_output -= overturn_value * units::sgn(linear_output);
            }
        }

        // apply max voltage constraints
        if constexpr (hasLinearVoltageClamp<ControllersType>) {
            linear_output =
              this->controllers.linear_voltage_clamp.applyMax(linear_output);
        }
        if constexpr (hasAngularVoltageClamp<ControllersType>) {
            angular_output =
              this->controllers.angular_voltage_clamp.applyMax(angular_output);
        }

        // apply slew
        if constexpr (hasLinearSlew<ControllersType>) {
            linear_output =
              this->controllers.linear_slew.apply(linear_output, delta_time);
        }
        if constexpr (hasAngularSlew<ControllersType>) {
            angular_output =
              this->controllers.angular_slew.apply(angular_output, delta_time);
        }

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    Ramsete(ControllersType controllers,
            Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
            mp::Trajectory* target_trajectory,
            zeta_units zeta,
            beta_units beta)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target_trajectory(target_trajectory),
          zeta(zeta),
          beta(beta) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    Ramsete(ControllersType controllers,
            Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
            mp::Trajectory* target_trajectory,
            double zeta,
            double beta)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target_trajectory(target_trajectory),
          zeta(zeta),
          beta(beta) {}

    Ramsete& getReference() {
        return *this;
    }

    // changer methods

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto reverse() {
        this->reversed = true;

        return this->getReference();
    }
};

} // namespace lyfast
} // namespace blazing
