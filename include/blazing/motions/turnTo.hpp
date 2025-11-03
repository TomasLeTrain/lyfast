#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/controllers/voltage_clamp.hpp"
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
         typename TolerancesType>
    requires angleTracker<TrackerType> && angularVelocityTracker<TrackerType> &&
               ArcadeDrivetrain<DrivetrainType> &&
               hasAngularFeedback<ControllersType>
class turnTo : public Motion<ControllersType,
                             DrivetrainType,
                             TrackerType,
                             TolerancesType>,
               public AngularMotion {
  private:
    // std::optional<units::V2Position> target_point = std::nullopt;
    // std::optional<Angle> given_target_heading = std::nullopt;
    std::variant<Angle, units::V2Position> target;

    // turnTo-specific properties
    std::optional<Time> m_timeout = std::nullopt;
    bool reversed = false;
    std::optional<AngularDirection> m_direction = std::nullopt;

    std::optional<TurnToState> m_state;

    // radius / track_width
    Number ratio = 0.0;

  public:
    int getLoopDelayTime() override {
        return 10;
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

        // defalts to std::nullopt if tracker does not implements getPosition
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

            const Angle directed_error =
              angleError(target_heading, heading, m_direction);

            // check for sign change in directionless error, if so then settling
            if (state.prev_directionless_error && state.prev_directed_error &&
                // if this is not true it might cross signs on the opposite side
                units::abs(*state.prev_directed_error) < 180_stDeg &&
                units::sgn(directionless_error) !=
                  units::sgn(*state.prev_directionless_error)) {
                state.settling = true;
            }

            state.prev_directionless_error = directionless_error;
            state.prev_directed_error = directed_error;

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
        result.inChainTolerance = result.inChainTolerance
                                    .transform([&](auto tolerance) {
                                        return tolerance | state.settling;
                                    })
                                    .value_or(false);

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
        Voltage linear_output = units::abs(angular_output) * ratio;

        // apply linear constraints and slew
        if constexpr (hasLinearVoltageClamp<ControllersType>) {
            linear_output =
              this->controllers.linear_voltage_clamp.apply(linear_output);
        }

        // apply slew
        if constexpr (hasLinearSlew<ControllersType>) {
            linear_output =
              this->controllers.linear_slew.apply(linear_output, delta_time);
        }

        this->drivetrain.moveArcade(linear_output, angular_output);

        return result;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Length x,
           Length y)
      // requires tracker to be able to track position without making it a
      // requirement for target heading
        requires positionTracker<TrackerType>
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target(units::V2Position(x, y)) {}

    // turnTo(ControllersType controllers,
    //        Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
    //        double x,
    //        double y)
    //     : turnTo(controllers, chassis, from_in(x), from_in(y)) {}
    //
    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Angle target_heading)
        : Motion<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis),
          target(target_heading) {}

    //
    // turnTo(ControllersType controllers,
    //        Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
    //        double target_heading)
    //     : turnTo(controllers, chassis, from_stDeg(target_heading)) {}

    turnTo& getReference() {
        return *this;
    }

    // changer methods

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto reverse() {
        this->reversed = true;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto radius(Number ratio = 1.0) {
        this->ratio = ratio;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto timeout(Time timeout) {
        this->m_timeout = timeout;

        return this->getReference();
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    auto direction(std::optional<AngularDirection> direction) {
        this->m_direction = direction;

        return this->getReference();
    }
};

// done as a different class to be able to inherit from linear motion
template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires angleTracker<TrackerType> && angularVelocityTracker<TrackerType> &&
               ArcadeDrivetrain<DrivetrainType> &&
               hasAngularFeedback<ControllersType> &&
               hasLinearFeedback<ControllersType>
class arc : public turnTo<ControllersType,
                          DrivetrainType,
                          TrackerType,
                          TolerancesType>,
            public LinearMotion {
  public:
    [[nodiscard("motion won't be executed unless run or async are used!")]]
    arc(ControllersType controllers,
        Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
        Angle target_heading,
        double radius = 1.0)
        : turnTo<ControllersType, DrivetrainType, TrackerType, TolerancesType>(
            controllers,
            chassis,
            target_heading) {
        // set radius (avoids nodiscard warning)
        std::ignore = this->radius(radius);
    }
};
} // namespace blazing
