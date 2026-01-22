#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "lyfast/motion_profiling/mp.hpp"
#include "lyfast/vel_controller.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

struct RamseteState {
    std::optional<Time> last_time;
    Time start_time;
    Length start_distance;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && forwardTravelTracker<TrackerType> &&
               TankDrivetrain<DrivetrainType> &&
               hasVelocityFeedforward<ControllersType>
class Ramsete : public Motion<ControllersType,
                              DrivetrainType,
                              TrackerType,
                              TolerancesType>,
                public LinearMotion,
                public AngularMotion {
  private:
    using zeta_units = Divided<Number, Angle>;
    using beta_units = Exponentiated<Divided<Angle, Length>, std::ratio<2>>;

    std::optional<RamseteState> m_state;
    bool reversed = false;

    mp::Trajectory* target_trajectory;

    // zeta = 1 / rad
    const Divided<Number, Angle> zeta = 1 / rad;
    // beta = rad^2 / length^2
    const Exponentiated<Divided<Angle, Length>, std::ratio<2>> beta =
      0.5 * units::pow<2>(rad / m);

    std::optional<Time> m_timeout = std::nullopt;
    Length close_threshold = 4_in;

  public:
    int getLoopDelayTime() override {
        return 10;
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

        RamseteState& state = m_state.value();
        motionExecutionResult result;

        Time delta_time = deltaTime(state.last_time);

        const units::V2Position position = this->tracker.getPosition();
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            // TODO: maybe reversing should be part of the motion?
            return reversed ? reverseAngle(heading) : heading;
        }();

        int target_idx =
          // target_trajectory->get_index_by_distance(
          //        units::max(0_in,
          //                   this->tracker.getForwardTravel() -
          //                   state.start_distance));
          target_trajectory->findClosestPointIndex(position);

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

        // k = 1 / time
        const Frequency k =
          2.0 * zeta *
          units::sqrt(units::square(target_speeds.angular_velocity) +
                      beta * units::square(target_speeds.linear_velocity));

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

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto closeThreshold(Length threshold) {
        this->close_threshold = threshold;
        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto timeout(Time timeout) {
        this->m_timeout = timeout;

        return this->getReference();
    }

}; // namespace lyfast

} // namespace lyfast
} // namespace blazing
