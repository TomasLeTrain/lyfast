#pragma once

#include "blazing/controllers/clamp.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <optional>

namespace blazing {

struct DistanceAtHeadingState {
    Length initial_forward_travel;

    Time start_time;
    std::optional<Time> last_time;

    bool linear_settled;
    bool angular_settled;

    bool settling;

    std::optional<Angle> prev_directionless_error;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires velocityTracker<TrackerType> &&
               forwardTravelTracker<TrackerType> &&
               ArcadeDrivetrain<DrivetrainType> &&
               hasAngularFeedback<ControllersType> &&
               hasLinearFeedback<ControllersType>
class distanceAtHeading
    : public Motion<ControllersType,
                    DrivetrainType,
                    TrackerType,
                    TolerancesType,
                    distanceAtHeading<ControllersType,
                                      DrivetrainType,
                                      TrackerType,
                                      TolerancesType>>,
      public LinearMotion<distanceAtHeading<ControllersType,
                                            DrivetrainType,
                                            TrackerType,
                                            TolerancesType>>,
      public AngularMotion<distanceAtHeading<ControllersType,
                                             DrivetrainType,
                                             TrackerType,
                                             TolerancesType>> {
  private:
    Length target_distance;
    std::optional<Angle> given_target_heading = std::nullopt;

    // distanceAtHeading-specific properties
    bool reversed = false;
    std::optional<Time> m_timeout = std::nullopt;
    std::optional<AngularDirection> m_direction = std::nullopt;

    bool m_velocity_based = false;

    std::optional<DistanceAtHeadingState> m_state;

  public:
    int getLoopDelayTime() override {
        return 10;
    }

    std::optional<motionExecutionResult> execute() override {
        if (!m_state.has_value()) {
            m_state = { .initial_forward_travel =
                          this->tracker->getForwardTravel(),
                        .start_time = now(),
                        .last_time = now(),
                        .linear_settled = false,
                        .angular_settled = false,
                        .settling = false,
                        .prev_directionless_error = std::nullopt };

            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        DistanceAtHeadingState& state = m_state.value();
        motionExecutionResult result;

        // should never equal 0_sec
        Time delta_time = deltaTime(state.last_time);

        Length forward_travel = this->tracker->getForwardTravel();
        const Angle heading = [&] {
            const Angle heading = this->tracker->getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        Angle target_heading =
          // use given target heading
          given_target_heading
            // or just target current angle so that the angle doesn't move at
            // all
            .value_or(heading);

        if (reversed) target_distance *= -1.0;

        Length linear_error =
          (target_distance + state.initial_forward_travel) - forward_travel;

        const Angle angular_error = [&] -> Angle {
            const Angle directionless_error =
              angleError(target_heading, heading);

            // check for sign change in directionless error, if so then settling
            if (!state.settling && state.prev_directionless_error &&
                units::sgn(directionless_error) !=
                  units::sgn(*state.prev_directionless_error)) {
                state.settling = true;
            }

            state.prev_directionless_error = directionless_error;

            return state.settling ?
                     directionless_error :
                     angleError(target_heading, heading, m_direction);
        }();

        // linear tolerances
        this->tolerances.linearErrorToleranceUpdate(linear_error);
        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker->getLinearVelocity());

        // angular tolerances
        this->tolerances.angularErrorToleranceUpdate(angular_error);
        this->tolerances.angularVelocityToleranceUpdate(
          this->tracker->getAngularVelocity());

        auto updateTolerance = [](std::optional<bool>& tolerance,
                                  bool curr_in_tolerance) {
            // if tolerance exists then it gets anded with curr_in_tolerance
            // else it gets set to curr_in_tolerance
            tolerance = tolerance.value_or(true) && curr_in_tolerance;
        };

        // get set to true if either normal/large tolerances are finished
        state.angular_settled = false;
        state.linear_settled = false;

        // check tolerances
        if constexpr (hasLinearTolerance<TolerancesType>) {
            updateTolerance(result.inSmallTolerance,
                            this->tolerances.linear.withinTolerance());

            state.linear_settled |= this->tolerances.linear.finished();
        }
        if constexpr (hasLargeLinearTolerance<TolerancesType>) {
            updateTolerance(result.inLargeTolerance,
                            this->tolerances.large_linear.withinTolerance());

            state.linear_settled |= this->tolerances.large_linear.finished();
        }
        if constexpr (hasAngularTolerance<TolerancesType>) {
            updateTolerance(result.inSmallTolerance,
                            this->tolerances.angular.withinTolerance());

            state.angular_settled |= this->tolerances.angular.finished();
        }
        if constexpr (hasLargeAngularTolerance<TolerancesType>) {
            updateTolerance(result.inLargeTolerance,
                            this->tolerances.large_angular.withinTolerance());

            state.angular_settled |= this->tolerances.large_angular.finished();
        }

        // dont use to check if we have finished
        if constexpr (hasChainLinearTolerance<TolerancesType>) {
            updateTolerance(result.inChainTolerance,
                            this->tolerances.chain_linear.withinTolerance());
        }
        if constexpr (hasChainAngularTolerance<TolerancesType>) {
            updateTolerance(result.inChainTolerance,
                            this->tolerances.chain_angular.withinTolerance());
        }

        result.finished = state.linear_settled && state.angular_settled;

        // check timeout
        result.finished |= timeoutDone(m_timeout, state.start_time);

        // finished if any of the available tolerances or timeout are
        // triggered
        if (result.finished) {
            this->drivetrain->moveArcade(0_volt, 0_volt);
            // returns immediately to avoid more movement
            return result;
        }

        // only evaluate velocity based if we have all the requirements
        if constexpr (hasLinearVelocityFeedback<ControllersType> &&
                      hasAngularVelocityFeedback<ControllersType> &&
                      VelocityArcadeDrivetrain<DrivetrainType>) {
            if (m_velocity_based) {
                LinearVelocity linear_vel =
                  this->controllers.linear_velocity_feedback.update(
                    -linear_error,
                    0_in,
                    delta_time);

                AngularVelocity angular_vel =
                  this->controllers.angular_velocity_feedback.update(
                    -angular_error,
                    0_stRad,
                    delta_time);

                if constexpr (hasLinearVelocityClamp<ControllersType>) {
                    linear_vel =
                      this->controllers.linear_velocity_clamp.apply(linear_vel);
                }
                if constexpr (hasAngularVelocityClamp<ControllersType>) {
                    angular_vel =
                      this->controllers.angular_velocity_clamp.apply(
                        angular_vel);
                }

                // don't apply slew when settling
                if (!state.settling) {
                    if constexpr (hasLinearVelocitySlew<ControllersType>) {
                        linear_vel =
                          this->controllers.linear_velocity_slew.apply(
                            linear_vel,
                            delta_time);
                    }
                    if constexpr (hasAngularVelocitySlew<ControllersType>) {
                        angular_vel =
                          this->controllers.angular_velocity_slew.apply(
                            angular_vel,
                            delta_time);
                    }
                }

                DifferentialSpeeds target { linear_vel, angular_vel };

                this->drivetrain->moveArcade(target.linear_velocity,
                                             target.angular_velocity);
                return result;

                // pass velocities into feedforward
                // auto [left_voltage, right_voltage] =
                //   this->controllers.velocity_feedforward.update(target,
                //                                                 delta_time);
                //
                // // TODO: apply voltage clamp/slew? probably not
                //
                // auto [left_vel, right_vel] =
                //   this->drivetrain->getDrivetrainVelocities();
                // auto [actual_volt_left, actual_volt_right] =
                //   this->drivetrain->getDrivetrainVoltages();
                //
                // // std::cout << std::fixed;
                // // std::cout << std::setprecision(5);
                // //
                // // std::cout << "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/"
                // //              "av_l/av_r/x/y/theta/t_err: "
                // //           << linear_error.internal() << " "
                // //           << target.linear_velocity.internal() << " "
                // //           << target.angular_velocity.internal() << " "
                // //           << left_vel.internal() << " " <<
                // //           right_vel.internal()
                // //           << " " << left_voltage.internal() << " "
                // //           << right_voltage.internal() << " "
                // //           << actual_volt_left.internal() << " "
                // //           << actual_volt_right.internal() << " "
                // //           << position.x.convert(in) << " "
                // //           << position.y.convert(in) << " "
                // //           << projected_cte_error.convert(in) << " "
                // //           << angular_error.internal() << std::endl;
                //
                // this->drivetrain->moveTank(left_voltage, right_voltage);
                //
                // // we return here, so none of the below code executes
                // return result;
            } else {
                // assert to warn user?
                // assert("want to use velocity but don't have requirements!");
            }
        }

        Voltage angular_output =
          this->controllers.angular_feedback.update(-angular_error,
                                                    0_stRad,
                                                    delta_time);

        Voltage linear_output =
          this->controllers.linear_feedback.update(-linear_error,
                                                   0_in,
                                                   delta_time);

        // apply voltage constraints
        if constexpr (hasLinearVoltageClamp<ControllersType>) {
            linear_output =
              this->controllers.linear_voltage_clamp.apply(linear_output);
        }
        if constexpr (hasAngularVoltageClamp<ControllersType>) {
            angular_output =
              this->controllers.angular_voltage_clamp.apply(angular_output);
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

        this->drivetrain->moveArcade(linear_output, angular_output);

        return result;
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    distanceAtHeading(
      ControllersType controllers,
      Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
      Length target_distance)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 distanceAtHeading<ControllersType,
                                   DrivetrainType,
                                   TrackerType,
                                   TolerancesType>>(controllers, chassis),
          target_distance(target_distance) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    distanceAtHeading(
      ControllersType controllers,
      Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
      double target_distance)
        : distanceAtHeading(controllers, chassis, from_in(target_distance)) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    distanceAtHeading(
      ControllersType controllers,
      Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
      Length target_distance,
      Angle target_heading)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 distanceAtHeading<ControllersType,
                                   DrivetrainType,
                                   TrackerType,
                                   TolerancesType>>(controllers, chassis),
          target_distance(target_distance),
          given_target_heading(target_heading) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    distanceAtHeading(
      ControllersType controllers,
      Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
      double target_distance,
      double target_heading)
        : distanceAtHeading(controllers,
                            chassis,
                            from_in(target_distance),
                            from_stDeg(target_heading)) {}

    // changer methods
    motionChangerMsg distanceAtHeading& reverse() {
        this->reversed = true;

        return *this;
    }

    motionChangerMsg distanceAtHeading& timeout(Time timeout) {
        this->m_timeout = timeout;

        return *this;
    }

    motionChangerMsg distanceAtHeading&
    direction(std::optional<AngularDirection> direction) {
        this->m_direction = direction;

        return *this;
    }

    motionChangerMsg distanceAtHeading& velocity_based(bool velocity_based) {
        this->m_velocity_based = velocity_based;

        return *this;
    }
};
} // namespace blazing
