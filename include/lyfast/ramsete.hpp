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
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType>
    requires poseTracker<TrackerType> && linearVelocityTracker<TrackerType> &&
               ArcadeDrivetrain<DrivetrainType> &&
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

    std::optional<Ramsete> m_state;
    bool reversed = false;

    mp::Trajectory* target_trajectory;

    // zeta = 1 / rad
    const Divided<Number, Angle> zeta = 1 / rad;
    // beta = rad^2 / length^2
    const Exponentiated<Divided<Angle, Length>, std::ratio<2>> beta =
      0.5 * units::pow<2>(rad / m);

  public:
    int getLoopDelayTime() override {
        return 10;
    }

    std::optional<motionExecutionResult> execute() override {
        if (!m_state.has_value()) {
            m_state = { .last_time = now(), .start_time = now() };
            // done to prevent values like delta_time being 0
            return std::nullopt;
        }

        RamseteState& state = m_state.value();
        motionExecutionResult result;

        units::Pose target;

        // should never equal 0_sec
        Time delta_time = deltaTime(state.last_time);

        const units::V2Position position = this->tracker.getPosition();
        const Angle heading = [&] -> Angle {
            const Angle heading = this->tracker.getAngle();
            // TODO: maybe reversing should be part of the motion?
            return reversed ? reverseAngle(heading) : heading;
        }();

        units::V2Position local_error = (target - position).rotatedBy(heading);

        DifferentialSpeeds target_speeds =
          target_trajectory->get_by_time(now() - state.start_time);

        double reverse_multiplier = reversed ? -1.0 : 1.0;

        // reverse linear_velocity if needed
        target_speeds.linear_velocity *= reverse_multiplier;
        // TODO: angular_velocity from target speeds might be different
        // direction?

        Angle errorAngle = angleError(target.orientation, heading);

        // k = 1 / time
        const Frequency k =
          2.0 * zeta *
          units::sqrt(units::square(target_speeds.angular_velocity) +
                      beta * units::square(target_speeds.linear_velocity));

        DifferentialSpeeds new_speeds;

        // v_new = cos(e_theta) * v + k * e_x;
        new_speeds.linear_velocity =
          units::cos(errorAngle) * target_speeds.linear_velocity +
          k * local_error.x;

        // w_new = w + k * e_theta + beta * v * sinc(e_theta) * y;
        new_speeds.angular_velocity = target_speeds.angular_velocity +
                                      k * errorAngle +
                                      beta * target_speeds.linear_velocity *
                                        sinc(errorAngle) * local_error.y;

        DifferentialVoltages voltages =
          this->controllers.velocity_feedforward.update(new_speeds, delta_time);

        // check that we haven't finished the path timewise
        result.finished |=
          timeoutDone(target_trajectory->getTotalTime(), state.start_time);

        this->drivetrain.moveVoltage(voltages.left_voltage,
                                     voltages.right_voltage);

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
