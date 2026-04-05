#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers/clamp.hpp"
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
#include "units/units.hpp"
#include <functional>
#include <iostream>
#include <variant>

namespace blazing {
struct BoomerangState {
    std::optional<Time> last_time;
    Time start_time;

    bool crossed_sideways;
    bool close;

    std::optional<Length> initial_side;

    units::V2FPosition prev_position;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && linearVelocityTracker<TrackerType> &&
               ArcadeDrivetrain<DrivetrainType> &&
               hasAngularFeedback<ControllersType> &&
               hasLinearFeedback<ControllersType>
class boomerang : public Motion<ControllersType,
                                DrivetrainType,
                                TrackerType,
                                TolerancesType,
                                boomerang<ControllersType,
                                          DrivetrainType,
                                          TrackerType,
                                          TolerancesType>>,
                  public LinearMotion<boomerang<ControllersType,
                                                DrivetrainType,
                                                TrackerType,
                                                TolerancesType>>,
                  public AngularMotion<boomerang<ControllersType,
                                                 DrivetrainType,
                                                 TrackerType,
                                                 TolerancesType>> {
  private:
    using pose_func_t = std::function<units::Pose()>;
    std::variant<units::Pose, pose_func_t> target;

    // boomerang-specific properties
    std::optional<Time> m_timeout = std::nullopt;
    bool reversed = false;
    double m_lead = 0.5;
    double m_lead2 = 0.0;

    Length close_threshold = 4_in;
    Length lead2_dist_threshold = 10_in;

    std::optional<Voltage> max_overturn_output = std::nullopt;

    std::optional<Divided<Angle, Length>> m_k_lat = std::nullopt;
    bool k_lat_only_settling = false;

    bool m_velocity_based = false;

    // defaults to cosine of angle
    std::function<double(Angle)> angular_linear_func =
      [](Angle angle) -> double {
        return units::cos(angle);
    };

    std::optional<BoomerangState> m_state;

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
            m_state = { .last_time = now(),
                        .start_time = now(),
                        .crossed_sideways = false,
                        .close = false,
                        .initial_side = std::nullopt,
                        .prev_position = this->tracker->getPosition() };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        BoomerangState& state = m_state.value();
        motionExecutionResult result;

        // should never equal 0_sec
        Time delta_time = deltaTime(state.last_time);

        const units::V2Position position = this->tracker->getPosition();

        const Angle heading = [&] {
            const Angle heading = this->tracker->getAngle();
            return reversed ? reverseAngle(heading) : heading;
        }();

        units::Pose target_pose = std::holds_alternative<units::Pose>(target) ?
                                    // either a target pose
                                    get<units::Pose>(target) :
                                    // or custom function returning a pose
                                    get<pose_func_t>(target)();

        // takes reverse into account
        // const Angle target_orientation =
        //   reversed ? reverseAngle(target.orientation) : target.orientation;
        const Angle target_orientation = target_pose.orientation;

        const Length pose_target_distance = position.distanceTo(target_pose);

        const units::V2Position carrot = [&] -> units::V2Position {
            if (state.close) return target_pose;
            auto carrot = target_pose - units::V2Position::fromPolar(
                                          target_orientation,
                                          pose_target_distance * m_lead);

            // // lead2 not active anymore, use normal carrot
            // if (pose_target_distance < lead2_dist_threshold || m_lead2 ==
            // 0.0)
            //     return carrot;

            // sideways error relative to the target angle
            // used to determine of to use lead2 or not
            Length sideways_error =
              (target_pose - position) *
              units::Vector2D { -units::sin(target_orientation),
                                units::cos(target_orientation) };

            if (!state.initial_side) state.initial_side = sideways_error;

            Length abs_sideways_error = units::abs(sideways_error);

            if (state.crossed_sideways ||
                abs_sideways_error < lead2_dist_threshold) {
                state.crossed_sideways = true;

                // return carrot;
                // if crossed we likely just want mtp behavior
                return target_pose;
            }

            if (m_lead2 == 0.0) {
                return carrot;
            }

            // perpendicular to lead
            auto lead2_vector =
              units::V2Position::fromPolar(target_orientation + 90_stDeg,
                                           pose_target_distance * m_lead2);

            // when the robot crosses the side, the carrot which is good
            // reverses we can keep track of the sign of vertical error to see
            // what we have to do

            auto carrot1 = carrot - lead2_vector;
            auto carrot2 = carrot + lead2_vector;

            bool swap_sides = false;

            if (units::sgn(*state.initial_side) != units::sgn(sideways_error)) {
                swap_sides = true;
            }

            // use carrot which minimizes distance
            // if we are swaping sides then it uses the maximum distance
            if (swap_sides ^
                (position.distanceTo(carrot1) < position.distanceTo(carrot2))) {
                return carrot1;
            } else {
                return carrot2;
            }
        }();

        // components of local error vector
        auto [forward_error, crosstrack_error] =
          (carrot - position).rotatedBy(-heading);

        // Length linear_error =
        //   position.distanceTo(carrot) * (reversed ? -1.0 : 1.0);
        Length linear_error = [&] -> Length {
            double reverse_multiplier = reversed ? -1.0 : 1.0;

            // use forward error when settling
            if (state.close) {
                return units::abs(forward_error) * reverse_multiplier;
            }

            // none active, error like normal
            return position.distanceTo(carrot) * reverse_multiplier;
        }();

        // when close gets activated it switches to move to point behavior
        if (units::abs(linear_error) < close_threshold && !state.close) {
            state.close = true;
        }

        Angle position_carrot_heading = position.angleTo(carrot);

        const Angle target_heading =
          state.close ? target_orientation : position_carrot_heading;

        Angle angular_error = angleError(target_heading, heading);

        // why is this different???
        Angle position_carrot_error =
          angleError(position_carrot_heading, heading);

        // used for cosine scaling and applying correct sign for linear
        // error/output
        Number lin_multiplier = angular_linear_func(position_carrot_error);

        // applies sign component here so that sign of error is accurate
        linear_error *= signed_sgn(lin_multiplier);

        // update tolerances if they are included
        this->tolerances.linearErrorToleranceUpdate(linear_error);
        this->tolerances.linearVelocityToleranceUpdate(
          this->tracker->getLinearVelocity());
        this->tolerances.linearHalfcircleToleranceUpdate(
          position,
          target_pose,
          target_pose.orientation);

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
            this->drivetrain->moveArcade(0_volt, 0_volt);
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
                  this->drivetrain->getDrivetrainVelocities();
                auto [actual_volt_left, actual_volt_right] =
                  this->drivetrain->getDrivetrainVoltages();

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

                this->drivetrain->moveTank(left_voltage, right_voltage);

                // we return here, so none of the below code executes
                return result;
            } else {
                // assert to warn user?
                // assert("want to use velocity but don't have
                // requirements!");
            }
        }

        Voltage angular_output =
          this->controllers.angular_feedback.update(-angular_error,
                                                    0_stRad,
                                                    delta_time);

        Voltage linear_output =
          this->controllers.linear_feedback.update(-linear_error,
                                                   0.0_in,
                                                   delta_time);

        if (m_k_lat && (!k_lat_only_settling ||
                        (k_lat_only_settling && state.crossed_sideways))) {
            angular_output = angular_output +
                             *m_k_lat * linear_output *
                               (target_pose - position).rotatedBy(-heading).y *
                               sinc(angular_error);
        }

        // sign was already applied to error, only applies cosine scaling
        // component
        linear_output *= units::abs(lin_multiplier);

        // here the robot would attempt to move backwards, when instead the
        // robot should turn around until it should start moving towards the
        // target
        // the reason that this is done to linear_output and not
        // linear_error is because that would trigger error tolerances
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

        this->drivetrain->moveArcade(linear_output, angular_output);

        return result;
    }

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    boomerang(ControllersType controllers,
              Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
              units::Pose pose)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 boomerang<ControllersType,
                           DrivetrainType,
                           TrackerType,
                           TolerancesType>>(controllers, chassis),
          target(pose) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    boomerang(ControllersType controllers,
              Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
              pose_func_t pose_func)
        : Motion<ControllersType,
                 DrivetrainType,
                 TrackerType,
                 TolerancesType,
                 boomerang<ControllersType,
                           DrivetrainType,
                           TrackerType,
                           TolerancesType>>(controllers, chassis),
          target(pose_func) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    boomerang(ControllersType controllers,
              Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
              Length x,
              Length y,
              Angle heading)
        : boomerang(controllers, chassis, { x, y, heading }) {}

    [[nodiscard("motion won't be executed unless run or async are used!")]]
    boomerang(ControllersType controllers,
              Chassis<DrivetrainType, TrackerType, TolerancesType> chassis,
              double x,
              double y,
              double heading)
        : boomerang(controllers,
                    chassis,
                    from_in(x),
                    from_in(y),
                    from_stDeg(heading)) {}

    // changer methods
    motionChangerMsg boomerang& reverse() {
        this->reversed = true;

        return *this;
    }

    motionChangerMsg boomerang&
    withOverturn(Voltage max_overturn_output = 1_volt) {
        this->max_overturn_output = max_overturn_output;

        return *this;
    }

    motionChangerMsg boomerang& closeThreshold(Length threshold) {
        this->close_threshold = threshold;
        return *this;
    }

    motionChangerMsg boomerang& lead2DistThreshold(Length threshold) {
        this->lead2_dist_threshold = threshold;
        return *this;
    }

    motionChangerMsg boomerang& lead(double lead, double lead2 = 0.0) {
        this->m_lead = lead;
        this->m_lead2 = lead2;
        return *this;
    }

    motionChangerMsg boomerang& k_lat(
      std::optional<std::variant<Divided<Angle, Length>, double, int>> k_lat =
        std::nullopt,
      bool only_when_settling = true) {
        this->k_lat_only_settling = only_when_settling;

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

    motionChangerMsg boomerang& customAngularLinearFunc(
      std::function<double(Angle)> custom_angular_linear_func) {
        angular_linear_func = custom_angular_linear_func;
        return *this;
    }

    motionChangerMsg boomerang& timeout(Time timeout) {
        this->m_timeout = timeout;

        return *this;
    }

    motionChangerMsg boomerang& velocity_based(bool velocity_based) {
        this->m_velocity_based = velocity_based;

        return *this;
    }
};
} // namespace blazing
