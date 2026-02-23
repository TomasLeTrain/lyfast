#pragma once

#include "blazing/controllers/clamp.hpp"
#include "blazing/controllers/controllers.hpp"
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
#include <variant>

namespace blazing {

struct TurnToState {
    Time start_time;
    std::optional<Time> last_time;

    bool settled;
    bool settling;
    std::optional<Angle> prev_directionless_error;
    std::optional<Angle> prev_directed_error;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType,
         typename Derived>
    requires angleTracker<TrackerType> && angularVelocityTracker<TrackerType> &&
               ArcadeDrivetrain<DrivetrainType> &&
               hasAngularFeedback<ControllersType>
class turnToBase : public Motion<ControllersType,
                                 DrivetrainType,
                                 TrackerType,
                                 TolerancesType,
                                 Derived>,
                   public AngularMotion<Derived> {
  private:
    // std::optional<units::V2Position> target_point = std::nullopt;
    // std::optional<Angle> given_target_heading = std::nullopt;
    std::variant<Angle, units::V2Position> target;

    // turnTo-specific properties
    std::optional<Time> m_timeout = std::nullopt;
    bool reversed = false;
    std::optional<AngularDirection> m_direction = std::nullopt;

    std::optional<TurnToState> m_state;

    Length m_radius = 0.0_in;
    std::optional<LinearVelocity> constant_velocity;

    bool m_velocity_based = false;

  public:
    int getLoopDelayTime() override {
        // return 10;
        if (m_velocity_based) {
            // useful to make derivative not super bad
            return 20;
        } else {
            // TODO: should probably also switch this one out?
            return 10;
        }
    }

    std::optional<motionExecutionResult> execute() override {
        if (!m_state.has_value()) {
            m_state = { .start_time = now(),
                        .last_time = now(),
                        .settled = false,
                        .settling = false,
                        .prev_directionless_error = std::nullopt,
                        .prev_directed_error = std::nullopt };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        TurnToState& state = m_state.value();
        motionExecutionResult result;

        // should never equal 0_sec
        Time delta_time = deltaTime(state.last_time);

        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        // defaults to std::nullopt if tracker does not implements getPosition
        const std::optional<units::V2Position> position = [this] {
            if constexpr (positionTracker<TrackerType>)
                return this->tracker.getPosition();
            else
                return std::nullopt;
        }();

        if (std::holds_alternative<units::V2Position>(target) &&
            !position.has_value()) {
            // should not be possible
            printf("invalid configuration! target is point but tracker doesn't "
                   "track position!\n");
        }

        const Angle angular_error = [&] -> Angle {
            const Angle target_heading =
              std::holds_alternative<Angle>(target) ?
                std::get<Angle>(target) :
                position.value().angleTo(std::get<units::V2Position>(target));

            const Angle directionless_error =
              angleError(target_heading, heading);

            Angle directed_error =
              angleError(target_heading, heading, m_direction);

            // if motion has direction then:
            // state.prev_directed_error ~ 3_deg
            // directed_error ~ 360_deg
            //
            // state.prev_directionless_error ~ 3_deg
            // state.prev_directionless_error ~ -3_deg
            //
            // sign change of directionless error can signal settling, but only
            // if directed error is closer to zero (the prev error at least)

            // check for sign change in directionless error, if so then settling
            if (state.prev_directionless_error && state.prev_directed_error &&
                // highly unlikely it can cross signs and also be greater than
                // 160
                units::abs(*state.prev_directed_error) < 160_stDeg &&
                units::sgn(directionless_error) !=
                  units::sgn(*state.prev_directionless_error)) {
                state.settling = true;
            }

            state.prev_directionless_error = directionless_error;
            state.prev_directed_error = directed_error;

            // avoid oscilations when close to 180 error
            if (!state.settling && !m_direction.has_value() &&
                units::abs(directed_error) > 175_stDeg) {
                // prefer going positive direction
                if (directed_error < 0_stDeg) {
                    directed_error += rot;
                }
            }

            return state.settling ? directionless_error : directed_error;
        }();

        // update tolerances
        this->tolerances.angularErrorToleranceUpdate(angular_error);
        this->tolerances.angularVelocityToleranceUpdate(
          this->tracker.getAngularVelocity());

        state.settled = false;

        // check tolerances
        if constexpr (hasAngularTolerance<TolerancesType>) {
            result.inSmallTolerance =
              this->tolerances.angular.withinTolerance();
            state.settled |= this->tolerances.angular.finished();
        }
        if constexpr (hasLargeAngularTolerance<TolerancesType>) {
            result.inLargeTolerance =
              this->tolerances.large_angular.withinTolerance();
            state.settled |= this->tolerances.large_angular.finished();
        }
        if constexpr (hasChainAngularTolerance<TolerancesType>) {
            result.inChainTolerance =
              this->tolerances.chain_angular.withinTolerance();
            // doesn't get used to check if finished
        }

        // when chaining we would like to chain immediately
        result.inChainTolerance =
          result.inChainTolerance
            .transform([&](auto tolerance) {
                return tolerance | state.settling;
            })
            // no in chain tolerance, could still trigger with settling
            .value_or(state.settling);

        result.finished = state.settled;

        // check timeout
        result.finished |= timeoutDone(m_timeout, state.start_time);

        // finished if any of the available tolerances or timeout are
        // triggered
        if (result.finished) {
            this->drivetrain.moveArcade(0_volt, 0_volt);
            // returns immediately to avoid more movement
            return result;
        }

        // only evaluate velocity based if we have all the requirements
        if constexpr (hasLinearVelocityFeedback<ControllersType> &&
                      hasAngularVelocityFeedback<ControllersType> &&
                      TankDrivetrain<DrivetrainType> &&
                      // has velocity feedforward
                      requires(ControllersType controller) {
                          controller.velocity_feedforward;
                      }) {
            if (m_velocity_based) {
                AngularVelocity angular_vel =
                  this->controllers.angular_velocity_feedback.update(
                    -angular_error,
                    0_stRad,
                    delta_time);

                if constexpr (hasAngularVelocityClamp<ControllersType>) {
                    angular_vel =
                      this->controllers.angular_velocity_clamp.apply(
                        angular_vel);
                }

                if constexpr (hasAngularVelocitySlew<ControllersType>) {
                    angular_vel =
                      this->controllers.angular_velocity_slew.apply(angular_vel,
                                                                    delta_time);
                }

                // calculates linear based on the capped angular to keep ratio

                LinearVelocity linear_vel = 0_inps;
                if (constant_velocity.has_value()) {
                    linear_vel = constant_velocity.value();
                } else {
                    linear_vel = units::abs(angular_vel) * m_radius / rad;
                }

                // if constexpr (hasLinearVelocityClamp<ControllersType>) {
                //     linear_vel =
                //       this->controllers.linear_velocity_clamp.apply(linear_vel);
                // }
                //
                // // apply slew
                // if constexpr (hasLinearVelocitySlew<ControllersType>) {
                //     linear_vel =
                //       this->controllers.linear_velocity_slew.apply(linear_vel,
                //                                                    delta_time);
                // }

                DifferentialSpeeds target { linear_vel, angular_vel };

                // pass velocities into feedforward
                auto [left_voltage, right_voltage] =
                  this->controllers.velocity_feedforward.update(target,
                                                                delta_time);

                // TODO: apply voltage clamp/slew? probably not

                auto [left_vel, right_vel] =
                  this->drivetrain.getDrivetrainVelocities();
                auto [actual_volt_left, actual_volt_right] =
                  this->drivetrain.getDrivetrainVoltages();
                //

                // std::cout << std::fixed;
                // std::cout << std::setprecision(5);
                //
                // std::cout << "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/"
                //              "av_l/av_r/x/y/theta/t_err: "
                //           << angular_error.internal() << " "
                //           << target.linear_velocity.internal() << " "
                //           << target.angular_velocity.internal() << " "
                //           << left_vel.internal() << " " <<
                //           right_vel.internal()
                //           << " " << left_voltage.internal() << " "
                //           << right_voltage.internal() << " "
                //           << actual_volt_left.internal() << " "
                //           << actual_volt_right.internal() << " " << 0 << " "
                //           << 0 << " " << heading.convert(deg) << " "
                //           << angular_error.internal() << std::endl;

                this->drivetrain.moveTank(left_voltage, right_voltage);

                // we return here, so none of the below code executes
                return result;
            } else {
                // assert to warn user?
                // assert("want to use velocity but don't have requirements!");
            }
        }

        Voltage angular_output =
          this->controllers.angular_feedback.update(-angular_error,
                                                    0_stRad,
                                                    delta_time);

        // apply voltage constraints
        if constexpr (hasAngularVoltageClamp<ControllersType>) {
            angular_output =
              this->controllers.angular_voltage_clamp.apply(angular_output);
        }

        // apply slew
        if constexpr (hasAngularSlew<ControllersType>) {
            angular_output =
              this->controllers.angular_slew.apply(angular_output, delta_time);
        }

        // done after voltage constraints / slew
        Voltage linear_output =
          units::abs(angular_output) * m_radius.convert(in);

        // don't apply linear slew or clamp to keep ratio
        // slew and clamp on the angle should be used instead

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

    motionChangerMsg
    turnToBase(ControllersType controllers,
               Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
               Length x,
               Length y)
      // requires tracker to be able to track position without making it a
      // requirement for target heading
        requires positionTracker<TrackerType>
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 Derived>(controllers, chassis),
          target(units::V2Position(x, y)) {}

    motionChangerMsg
    turnToBase(ControllersType controllers,
               Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
               units::V2Position point)
      // requires tracker to be able to track position without making it a
      // requirement for target heading
        requires positionTracker<TrackerType>
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 Derived>(controllers, chassis),
          target(point) {}

    motionChangerMsg
    turnToBase(ControllersType controllers,
               Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
               Angle target_heading)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 Derived>(controllers, chassis),
          target(target_heading) {}

    //
    // turnTo(ControllersType controllers,
    //        Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
    //        double target_heading)
    //     : turnTo(controllers, chassis, from_stDeg(target_heading)) {}

    // changer methods - due to arc inhertance they have to be defined this way
    motionChanger reverse() {
        this->reversed = true;
        return DerivedReturnType;
    }

    motionChanger radius(Length radius) {
        this->m_radius = radius;
        return DerivedReturnType;
    }

    motionChanger constantVelocity(std::optional<LinearVelocity> vel) {
        this->constant_velocity = vel;

        return DerivedReturnType;
    }

    // allow setting in ratio mode
    motionChanger radius(Number radius) {
        this->m_radius = radius * in;
        return DerivedReturnType;
    }

    motionChanger timeout(Time timeout) {
        this->m_timeout = timeout;
        return DerivedReturnType;
    }

    motionChanger direction(std::optional<AngularDirection> direction) {
        this->m_direction = direction;
        return DerivedReturnType;
    }

    motionChanger velocity_based(bool velocity_based) {
        this->m_velocity_based = velocity_based;

        return DerivedReturnType;
    }
};

// done as a different class to be able to inherit from linear motion
template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
class Arc
    : public turnToBase<
        ControllersType,
        DrivetrainType,
        TrackerType,
        TolerancesType,
        Arc<ControllersType, DrivetrainType, TrackerType, TolerancesType>>,
      public LinearMotion<
        Arc<ControllersType, DrivetrainType, TrackerType, TolerancesType>> {
  public:
    [[nodiscard("motion won't be executed unless run or async are used!")]]
    Arc(ControllersType controllers,
        Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
        Angle target_heading,
        double radius = 1.0)
        : turnToBase<
            ControllersType,
            DrivetrainType,
            TrackerType,
            TolerancesType,
            Arc<ControllersType, DrivetrainType, TrackerType, TolerancesType>>(
            controllers,
            chassis,
            target_heading) {
        // set radius (avoids nodiscard warning)
        std::ignore = this->radius(radius);
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    Arc(ControllersType controllers,
        Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
        Length x,
        Length y,
        auto radius)
        : turnToBase<
            ControllersType,
            DrivetrainType,
            TrackerType,
            TolerancesType,
            Arc<ControllersType, DrivetrainType, TrackerType, TolerancesType>>(
            controllers,
            chassis,
            x,
            y) {
        // set radius (avoids nodiscard warning)
        std::ignore = this->radius(radius);
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    Arc(ControllersType controllers,
        Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
        units::V2Position target_point,
        auto radius)
        : turnToBase<
            ControllersType,
            DrivetrainType,
            TrackerType,
            TolerancesType,
            Arc<ControllersType, DrivetrainType, TrackerType, TolerancesType>>(
            controllers,
            chassis,
            target_point) {
        // set radius (avoids nodiscard warning)
        std::ignore = this->radius(radius);
    }
};

// simple wrapper for turnTo without the derived type
template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
class turnTo
    : public turnToBase<
        ControllersType,
        DrivetrainType,
        TrackerType,
        TolerancesType,
        turnTo<ControllersType, DrivetrainType, TrackerType, TolerancesType>> {
  public:
    motionChangerMsg
    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Length x,
           Length y)
        : turnToBase<ControllersType,
                     DrivetrainType,
                     TrackerType,
                     TolerancesType,
                     turnTo<ControllersType,
                            DrivetrainType,
                            TrackerType,
                            TolerancesType>>(controllers,
                                             chassis,
                                             units::V2Position(x, y)) {}

    motionChangerMsg
    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           units::V2Position point)
        : turnToBase<ControllersType,
                     DrivetrainType,
                     TrackerType,
                     TolerancesType,
                     turnTo<ControllersType,
                            DrivetrainType,
                            TrackerType,
                            TolerancesType>>(controllers, chassis, point) {}

    motionChangerMsg
    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Angle target_heading)
        : turnToBase<ControllersType,
                     DrivetrainType,
                     TrackerType,
                     TolerancesType,
                     turnTo<ControllersType,
                            DrivetrainType,
                            TrackerType,
                            TolerancesType>>(controllers,
                                             chassis,
                                             target_heading) {}
};
} // namespace blazing
