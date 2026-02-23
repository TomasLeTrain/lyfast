#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers/clamp.hpp"
#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedback/feedback.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/motions/motion.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/trackers/tracker.hpp"
#include "blazing/utils.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include <functional>
#include <iostream>
#include <optional>
#include <variant>

namespace blazing {
struct MoveToState {
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
               hasLinearFeedback<ControllersType>
class moveTo
    : public Motion<
        ControllersType,
        DrivetrainType,
        TrackerType,
        TolerancesType,
        moveTo<ControllersType, DrivetrainType, TrackerType, TolerancesType>>,
      public LinearMotion<
        moveTo<ControllersType, DrivetrainType, TrackerType, TolerancesType>>,
      public AngularMotion<
        moveTo<ControllersType, DrivetrainType, TrackerType, TolerancesType>> {
  private:
    using point_func_t = std::function<units::V2Position()>;
    std::variant<units::V2Position, point_func_t> target;

    // moveTo-specific properties
    std::optional<Time> m_timeout = std::nullopt;
    bool reversed = false;
    Length close_threshold = 7_in;
    std::optional<Voltage> max_overturn_output = std::nullopt;

    std::optional<Divided<Angle, Length>> m_k_lat = std::nullopt;

    bool m_only_x = false;
    bool m_only_y = false;

    std::optional<Length> m_custom_x_settling = std::nullopt;
    std::optional<Length> m_custom_y_settling = std::nullopt;

    bool m_velocity_based = false;

    // defaults to cosine of angle
    std::function<double(Angle)> angular_linear_func =
      [](Angle angle) -> double {
        return units::cos(angle);
    };

    std::optional<MoveToState> m_state;

  public:
    int getLoopDelayTime() override {
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
            m_state = {
                .close = false,
                .last_time = now(),
                .start_time = now(),
                .locked_heading = std::nullopt,
            };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        MoveToState& state = m_state.value();
        motionExecutionResult result;

        // should never equal 0_sec
        Time delta_time = deltaTime(state.last_time);

        const units::V2Position position = this->tracker.getPosition();
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        auto target_point = std::holds_alternative<units::V2Position>(target) ?
                              // either we have a point
                              std::get<units::V2Position>(target) :
                              // or a function returning a point
                              std::get<point_func_t>(target)();

        // components of local error vector
        auto [forward_error, crosstrack_error] =
          (target_point - position).rotatedBy(-heading);

        Length linear_error = [&] -> Length {
            double reverse_multiplier = reversed ? -1.0 : 1.0;

            // only go for x when settling
            if (m_only_x && state.close) {
                Length target_x = target_point.x;
                if (m_custom_x_settling.has_value())
                    target_x = *m_custom_x_settling;

                return units::abs(target_x - position.x) * reverse_multiplier;
            }
            // only go for x when settling
            if (m_only_y && state.close) {
                Length target_y = target_point.y;
                if (m_custom_y_settling.has_value())
                    target_y = *m_custom_y_settling;

                return units::abs(target_y - position.y) * reverse_multiplier;
            }

            // use forward error when settling
            if (state.close) {
                return units::abs(forward_error) * reverse_multiplier;
            }

            // none active, error like normal
            return (target_point - position).magnitude() * reverse_multiplier;
        }();

        Angle position_target_heading = position.angleTo(target_point);

        if (units::abs(linear_error) < close_threshold && !state.close) {
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
        // NOTE: sgn can be zero, which can set linear error to zero as well!
        linear_error *= signed_sgn(lin_multiplier);

        this->tolerances.linearErrorToleranceUpdate(linear_error);

        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker.getLinearVelocity());
        // TODO: does half circle exit make sense here?
        this->tolerances.linearHalfcircleToleranceUpdate(position,
                                                         target_point,
                                                         target_heading);

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

        // finished if any of the available tolerances or timeout are triggered
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

                // sign was already applied to error, only applies cosine
                // scaling component
                if (!state.close) linear_vel *= units::abs(lin_multiplier);

                // here the robot would attempt to move backwards, when instead
                // the robot should turn around until it should start moving
                // towards the target the reason that this is done to
                // linear_output and not linear_error is because otherwise
                // linear_error would be zero and tolerances would trigger
                if (!state.close && lin_multiplier < 0) {
                    linear_vel = 0_mps;
                }

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
                if (!state.close) {
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

                // pass velocities into feedforward
                auto [left_voltage, right_voltage] =
                  this->controllers.velocity_feedforward.update(target,
                                                                delta_time);

                // TODO: apply voltage clamp/slew? probably not

                auto [left_vel, right_vel] =
                  this->drivetrain.getDrivetrainVelocities();
                auto [actual_volt_left, actual_volt_right] =
                  this->drivetrain.getDrivetrainVoltages();

                // std::cout << std::fixed;
                // std::cout << std::setprecision(5);
                //
                // std::cout << "dist/lin/ang/drive_left/drive_right/tv_l/tv_r/"
                //              "av_l/av_r/x/y/theta/t_err: "
                //           << linear_error.internal() << " "
                //           << target.linear_velocity.internal() << " "
                //           << target.angular_velocity.internal() << " "
                //           << left_vel.internal() << " " <<
                //           right_vel.internal()
                //           << " " << left_voltage.internal() << " "
                //           << right_voltage.internal() << " "
                //           << actual_volt_left.internal() << " "
                //           << actual_volt_right.internal() << " "
                //           << position.x.convert(in) << " "
                //           << position.y.convert(in) << " "
                //           << projected_cte_error.convert(in) << " "
                //           << angular_error.internal() << std::endl;

                this->drivetrain.moveTank(left_voltage, right_voltage);

                // we return here, so none of the below code executes
                return result;
            } else {
                // assert to warn user?
                // assert("want to use velocity but don't have requirements!");
            }
        }

        // calculate outputs
        Voltage angular_output =
          this->controllers.angular_feedback.update(-angular_error,
                                                    0_stRad,
                                                    delta_time);

        Voltage linear_output =
          this->controllers.linear_feedback.update(-linear_error,
                                                   0.0_in,
                                                   delta_time);

        if (m_k_lat) {
            angular_output = angular_output +
                             *m_k_lat * linear_output *
                               (target_point - position).rotatedBy(-heading).y *
                               sinc(angular_error);
        }

        // sign was already applied to error, only applies cosine scaling
        // component
        linear_output *= units::abs(lin_multiplier);

        // here the robot would attempt to move backwards, when instead the
        // robot should turn around until it should start moving towards the
        // target
        // the reason that this is done to linear_output and not linear_error is
        // because otherwise linear_error would be zero and tolerances would
        // trigger
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

  public:
    [[nodiscard("motion won't be executed unless run or async are used!")]]
    moveTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           units::V2Position point)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 moveTo<ControllersType,
                        DrivetrainType,
                        TrackerType,
                        TolerancesType>>(controllers, chassis),
          target(point) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    moveTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           Length x,
           Length y)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 moveTo<ControllersType,
                        DrivetrainType,
                        TrackerType,
                        TolerancesType>>(controllers, chassis),
          target(units::V2Position(x, y)) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    moveTo(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
           point_func_t point_func)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 moveTo<ControllersType,
                        DrivetrainType,
                        TrackerType,
                        TolerancesType>>(controllers, chassis),
          target(point_func) {}

    // changer methods

    motionChangerMsg moveTo& reverse() {
        this->reversed = true;

        return *this;
    }

    motionChangerMsg moveTo& overturn(Voltage max_overturn_output = 1_volt) {
        this->max_overturn_output = max_overturn_output;

        return *this;
    }

    motionChangerMsg moveTo& k_lat(
      std::optional<std::variant<Divided<Angle, Length>, double, int>> k_lat =
        std::nullopt) {
        if (!k_lat)
            this->m_k_lat = std::nullopt;
        else {
            const auto& variant = k_lat.value();
            if (std::holds_alternative<Divided<Angle, Length>>(variant)) {
                this->m_k_lat = std::get<Divided<Angle, Length>>(variant);
            } else if (std::holds_alternative<double>(variant)) {
                this->m_k_lat = std::get<double>(variant) * (rad / m);
            } else if (std::holds_alternative<int>(variant)) {
                this->m_k_lat = std::get<int>(variant) * (rad / m);
            }
        }

        return *this;
    }

    motionChangerMsg moveTo& closeThreshold(Length threshold) {
        this->close_threshold = threshold;
        return *this;
    }

    motionChangerMsg moveTo& customAngularLinearFunc(
      std::function<double(Angle)> custom_angular_linear_func) {
        angular_linear_func = custom_angular_linear_func;
        return *this;
    }

    motionChangerMsg moveTo& timeout(Time timeout) {
        this->m_timeout = timeout;

        return *this;
    }

    motionChangerMsg moveTo&
    only_x(bool only_x,
           std::optional<Length> custom_x_settling = std::nullopt) {
        this->m_only_x = only_x;
        this->m_custom_x_settling = custom_x_settling;

        return *this;
    }

    motionChangerMsg moveTo&
    only_y(bool only_y,
           std::optional<Length> custom_y_settling = std::nullopt) {
        this->m_only_y = only_y;
        this->m_custom_y_settling = custom_y_settling;

        return *this;
    }

    motionChangerMsg moveTo& velocity_based(bool velocity_based) {
        this->m_velocity_based = velocity_based;

        return *this;
    }
};
} // namespace blazing
