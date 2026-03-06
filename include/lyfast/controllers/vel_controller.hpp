#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedforward/feedforward.hpp"
#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <functional>
#include <ios>

namespace blazing {
namespace lyfast {

// voltage is assumed to be in the range [0,1]

// u = Ks * sgn(v) + Kv * v + Ka * a;
using KsUnits = Voltage;
template<typename VelUnit>
using KvUnits = Divided<Voltage, VelUnit>;
template<typename VelUnit>
using KaUnits = Divided<Voltage, Divided<VelUnit, Time>>;

// both kv and kp are same units
template<typename VelUnit>
using KpUnits = Divided<Voltage, VelUnit>;
template<typename VelUnit>
using KiUnits = Divided<Voltage, Multiplied<VelUnit, Time>>;

using FKsUnits = FVoltage;
template<typename VelUnit>
using FKvUnits = ConvertFloatType<KvUnits<VelUnit>, float>;
template<typename VelUnit>
using FKaUnits = ConvertFloatType<KaUnits<VelUnit>, float>;

// both kv and kp are same units
template<typename VelUnit>
using FKpUnits = ConvertFloatType<KpUnits<VelUnit>, float>;
template<typename VelUnit>
using FKiUnits = ConvertFloatType<KiUnits<VelUnit>, float>;

// TODO: put into c++ file
inline DifferentialSpeeds
desaturatePrioritizeAngularDiffSpeeds(DifferentialSpeeds target,
                                      Length track_width,
                                      LinearVelocity max_velocity) {
    Length track_radius = track_width / 2.0;

    // first determine how fast we want to go angular wise
    LinearVelocity lin_alg_target =
      (target.angular_velocity / rad) * track_radius;

    // clamp linear speed based on angular speed
    LinearVelocity max_lin_speed =
      units::abs(max_velocity) - units::abs(lin_alg_target);

    LinearVelocity new_lin_speed =
      units::clamp(target.linear_velocity, -max_lin_speed, max_lin_speed);

    return { new_lin_speed, target.angular_velocity };
}

inline DifferentialSpeeds
desaturateDifferentialSpeeds(DifferentialSpeeds target,
                             Length track_width,
                             LinearVelocity max_velocity) {
    Length track_radius = track_width / 2.0;

    LinearVelocity target_left_vel =
      target.linear_velocity - (target.angular_velocity / rad) * track_radius;
    LinearVelocity target_right_vel =
      target.linear_velocity + (target.angular_velocity / rad) * track_radius;

    std::array<LinearVelocity, 2> saturated = { target_left_vel,
                                                target_right_vel };

    auto [new_left_vel, new_right_vel] =
      blazing::desaturate(saturated, max_velocity);

    LinearVelocity new_lin_vel = (new_left_vel + new_right_vel) / 2.0;
    AngularVelocity new_ang_vel =
      rad * (new_right_vel - new_left_vel) / track_width;

    return { new_lin_vel, new_ang_vel };
}

template<typename VelUnit>
struct FeedforwardVelocityControllerParams {
    KvUnits<VelUnit> Kv;
    KaUnits<VelUnit> Ka;
    Voltage Ks;

    // KvUnits<VelUnit> Kp { 0 };
    // Divided<Voltage, Multiplied<VelUnit, Time>> Ki { 0 };
    //
    // Voltage max_output { 1_volt };
    //
    // double tbh_factor { 0.0 };
};

template<typename VelUnit>
struct PIDVelocityControllerParams {
    KvUnits<VelUnit> Kp { 0 };
    Divided<Voltage, Multiplied<VelUnit, Time>> Ki { 0 };

    Voltage max_output { 1_volt };
    double tbh_factor { 0.0 };
};

template<typename VelUnit>
struct VelocityControllerParams {
    KvUnits<VelUnit> left_Kv;
    KaUnits<VelUnit> left_Ka;
    KsUnits left_Ks;
    KvUnits<VelUnit> left_Kp { 0 };
    Divided<Voltage, Multiplied<VelUnit, Time>> left_Ki { 0 };
    Voltage left_max_output { 1_volt };
    double left_tbh_factor { 0.0 };

    KvUnits<VelUnit> right_Kv;
    KaUnits<VelUnit> right_Ka;
    KsUnits right_Ks;
    KvUnits<VelUnit> right_Kp { 0 };
    Divided<Voltage, Multiplied<VelUnit, Time>> right_Ki { 0 };
    Voltage right_max_output { 1_volt };
    double right_tbh_factor { 0.0 };

    // construct both sides with equal gains
    static VelocityControllerParams fromFeedforwardPID(
      FeedforwardVelocityControllerParams<VelUnit> feedforward_params,
      PIDVelocityControllerParams<VelUnit> feedback_params) {
        return {
            .left_Kv = feedforward_params.Kv,
            .left_Ka = feedforward_params.Ka,
            .left_Ks = feedforward_params.Ks,
            .left_Kp = feedback_params.Kp,
            .left_Ki = feedback_params.Ki,
            .left_max_output = feedback_params.max_output,
            .left_tbh_factor = feedback_params.tbh_factor,

            .right_Kv = feedforward_params.Kv,
            .right_Ka = feedforward_params.Ka,
            .right_Ks = feedforward_params.Ks,
            .right_Kp = feedback_params.Kp,
            .right_Ki = feedback_params.Ki,
            .right_max_output = feedback_params.max_output,
            .right_tbh_factor = feedback_params.tbh_factor,
        };
    }
};

template<typename VelUnit>
class FeedforwardVelocityController {
  private:
    FeedforwardVelocityControllerParams<VelUnit> m_params;

    std::optional<VelUnit> last_speed = std::nullopt;

  public:
    Voltage updateKvKa(VelUnit target, Time duration) {
        Divided<VelUnit, Time> target_accel =
          (target -
           // combines measurement and last_speeds
           last_speed.value_or(VelUnit(0))) /
          duration;

        Voltage result { // kv
                         target * m_params.Kv +
                         // ka
                         target_accel * m_params.Ka
        };

        last_speed = { target };

        return result;
    }

    // allows applying ks later in the chain if other processes are done in
    // between
    Voltage applyKs(Voltage output) {
        // apply ks at the end
        return output + units::sgn(output) * m_params.Ks;
    }

    Voltage update(VelUnit target, Time duration) {
        Voltage result = updateKvKa(target, duration);
        result = applyKs(result);

        return result;
    }

    FeedforwardVelocityControllerParams<VelUnit> getParams() {
        return m_params;
    }

    void reset() {
        last_speed = std::nullopt;
    }

    void setParams(FeedforwardVelocityControllerParams<VelUnit> new_params) {
        m_params = new_params;
    }

    FeedforwardVelocityController(
      FeedforwardVelocityControllerParams<VelUnit> params)
        : m_params(params) {}
};

template<typename VelUnit>
class PIDVelocityController {
  private:
    PIDVelocityControllerParams<VelUnit> m_params;

    Multiplied<VelUnit, Time> integral { 0 };
    std::optional<VelUnit> last_error = std::nullopt;

    // local variables, membesr so they can be accessed between multiple methods
    Multiplied<VelUnit, Time> current_integral { 0 };
    VelUnit error;

  public:
    Voltage
    unclampedUpdate(VelUnit measurement, VelUnit target, Time duration) {
        error = target - measurement;

        current_integral = integral;

        if (last_error)
            // use trapezoidal approximation
            current_integral += (error + *last_error) * duration * 0.5;
        else
            // use Riemann sum approximation
            current_integral += error * duration;

        // decrease integral by some amount when crossing error to minimize
        // overshooot due to the integral
        if (last_error && units::sgn(error) != units::sgn(*last_error)) {
            current_integral *= m_params.tbh_factor;
            // update integral even if saturating?
            integral = current_integral;
        }

        Voltage result {
            // kp
            m_params.Kp * error +
              // ki
              m_params.Ki * current_integral,
        };

        last_error = error;

        return result;
    }

    // separated in case other components are added to the voltage (for example
    // feedforward terms)
    Voltage compensateForSaturation(Voltage result) {
        if (
          // currently saturating
          units::abs(result) >= m_params.max_output &&
          // output going in direction of error
          units::sgn(error) == units::sgn(result)) {
            // clamping, stop integral windup
            // no need to update integral to current integral
        } else {
            // not saturating, update integral
            integral = current_integral;
        }

        result =
          units::clamp(result, -m_params.max_output, m_params.max_output);

        return result;
    }

    Voltage update(VelUnit measurement, VelUnit target, Time duration) {
        Voltage result = unclampedUpdate(measurement, target, duration);

        result = compensateForSaturation(result);

        return result;
    }

    void reset() {
        integral = Multiplied<VelUnit, Time> { 0 };
        last_error = std::nullopt;
    }

    PIDVelocityControllerParams<VelUnit> getParams() {
        return m_params;
    }

    void setParams(PIDVelocityControllerParams<VelUnit> params) {
        m_params = params;
    }

    PIDVelocityController(PIDVelocityControllerParams<VelUnit> params)
        : m_params(params) {}
};

template<typename VelUnit>
class SimpleVelocityController {
    FeedforwardVelocityController<VelUnit> m_feedforward;

    // single pid for both possibilities?
    PIDVelocityController<VelUnit> m_pid;

  public:
    Voltage update(VelUnit measurement, VelUnit target, Time duration) {
        Voltage u_feedforward = m_feedforward.updateKvKa(target, duration);

        Voltage u_feedback =
          m_pid.unclampedUpdate(measurement, target, duration);

        Voltage result = u_feedforward + u_feedback;

        // apply ks after adding both feedback and feedforward pid
        result = m_feedforward.applyKs(result);

        // apply final pid step
        result = m_pid.compensateForSaturation(result);

        return result;
    }

    void reset() {
        m_feedforward.reset();
        m_pid.reset();
    }

    SimpleVelocityController(FeedforwardVelocityController<VelUnit> feedforward,
                             PIDVelocityController<VelUnit> pid)
        : m_feedforward(feedforward),
          m_pid(pid) {}
};

class DrivetrainSideVelocityController {
    struct TargetT {
        LinearVelocity linear;
        LinearVelocity angular;
    };

    FeedforwardVelocityController<LinearVelocity> m_linear;
    FeedforwardVelocityController<LinearVelocity> m_angular;

    // single pid for both possibilities?
    PIDVelocityController<LinearVelocity> m_pid;

  public:
    Voltage update(LinearVelocity measurement, TargetT target, Time duration) {
        LinearVelocity v_target = target.linear + target.angular;
        Voltage u_linear = m_linear.updateKvKa(target.linear, duration);
        Voltage u_angular = m_angular.updateKvKa(target.angular, duration);

        Voltage u_feedback =
          m_pid.unclampedUpdate(measurement, v_target, duration);

        Voltage result = u_linear + u_angular + u_feedback;

        // apply ks only from linear output
        result = m_linear.applyKs(result);

        // apply final pid step
        result = m_pid.compensateForSaturation(result);

        return result;
    }

    void reset() {
        m_linear.reset();
        m_angular.reset();
        m_pid.reset();
    }

    DrivetrainSideVelocityController(
      FeedforwardVelocityController<LinearVelocity> linear,
      FeedforwardVelocityController<LinearVelocity> angular,

      // single pid for both possibilities?
      PIDVelocityController<LinearVelocity> pid)
        : m_linear(linear),
          m_angular(angular),
          m_pid(pid) {}
};

class DifferentialVelocityController {
    DrivetrainSideVelocityController m_left_controller;
    DrivetrainSideVelocityController m_right_controller;

    LinearVelocity m_max_velocity;
    Length m_track_width;
    double m_vel_alpha = 1.0;
    bool m_prioritize_angular = false;

    // used for getting and filtering velocity
    std::reference_wrapper<DifferentialDrivetrain> drivetrain;
    std::optional<LeftRightSpeeds> last_velocities = std::nullopt;

  public:
    LeftRightVoltages update(LeftRightSpeeds measurement,
                             DifferentialSpeeds target,
                             Time duration) {
        Length track_radius = m_track_width / 2.0;

        // desaturate target first
        if (m_prioritize_angular)
            target = desaturatePrioritizeAngularDiffSpeeds(target,
                                                           m_track_width,
                                                           m_max_velocity);
        else
            target = desaturateDifferentialSpeeds(target,
                                                  m_track_width,
                                                  m_max_velocity);

        LinearVelocity target_linear_velocity = target.linear_velocity;
        // angular velocity converted to linear velocity wheel speeds
        LinearVelocity target_angular_velocity =
          (target.angular_velocity / rad) * track_radius;

        Voltage left_voltage =
          m_left_controller.update(measurement.left_vel,
                                   { .linear = target_linear_velocity,
                                     .angular = -target_angular_velocity },
                                   duration);

        Voltage right_voltage =
          m_right_controller.update(measurement.right_vel,
                                    { .linear = target_linear_velocity,
                                      .angular = target_angular_velocity },
                                    duration);

        LeftRightVoltages result = { left_voltage, right_voltage };

        return result;
    }

    LeftRightVoltages update(DifferentialSpeeds target, Time duration) {

        // fall back to using specified drivetrain
        LeftRightSpeeds velocities = drivetrain.get().getDrivetrainVelocities();

        // use low pass filter on the velocities
        // TODO: replace with kalman filter based velocity
        if (last_velocities) {
            velocities.left_vel =
              m_vel_alpha * velocities.left_vel +
              (1 - m_vel_alpha) * last_velocities.value().left_vel;

            velocities.right_vel =
              m_vel_alpha * velocities.right_vel +
              (1 - m_vel_alpha) * last_velocities.value().right_vel;
        }

        last_velocities = velocities;

        return update(velocities, target, duration);
    }

    void reset() {
        last_velocities = std::nullopt;
        m_left_controller.reset();
        m_right_controller.reset();
    }

    // return the left and right controllers
    std::pair<DrivetrainSideVelocityController,
              DrivetrainSideVelocityController>
    getControllers() {
        return { m_left_controller, m_right_controller };
    }

    void setLeftController(DrivetrainSideVelocityController controller) {
        m_left_controller = controller;
    }

    void setRightController(DrivetrainSideVelocityController controller) {
        m_right_controller = controller;
    }

    DifferentialVelocityController(
      DrivetrainSideVelocityController left_controller,
      DrivetrainSideVelocityController right_controller,
      LinearVelocity max_velocity,
      Length track_width,
      double vel_alpha,
      bool prioritize_angular,
      std::reference_wrapper<DifferentialDrivetrain> drivetrain)
        : m_left_controller(left_controller),
          m_right_controller(right_controller),
          m_max_velocity(max_velocity),
          m_track_width(track_width),
          m_vel_alpha(vel_alpha),
          m_prioritize_angular(prioritize_angular),
          drivetrain(drivetrain) {}
};

// explicit declarations
// feedforward
extern template struct FeedforwardVelocityControllerParams<LinearVelocity>;
extern template struct FeedforwardVelocityControllerParams<AngularVelocity>;

extern template class FeedforwardVelocityController<LinearVelocity>;
extern template class FeedforwardVelocityController<AngularVelocity>;

// PID
extern template struct PIDVelocityControllerParams<LinearVelocity>;
extern template struct PIDVelocityControllerParams<AngularVelocity>;

extern template class PIDVelocityController<LinearVelocity>;
extern template class PIDVelocityController<AngularVelocity>;

// simple vel controller
extern template class SimpleVelocityController<LinearVelocity>;
extern template class SimpleVelocityController<AngularVelocity>;

// using declarations to make it easier to work with
using LinearFeedforwardVelocityControllerParams =
  FeedforwardVelocityControllerParams<LinearVelocity>;
using AngularFeedforwardVelocityControllerParams =
  FeedforwardVelocityControllerParams<AngularVelocity>;

using LinearFeedforwardVelocityController =
  FeedforwardVelocityController<LinearVelocity>;
using AngularFeedforwardVelocityController =
  FeedforwardVelocityController<AngularVelocity>;

// PID
using LinearPIDVelocityControllerParams =
  PIDVelocityControllerParams<LinearVelocity>;
using AngularPIDVelocityControllerParams =
  PIDVelocityControllerParams<AngularVelocity>;

using LinearPIDVelocityController = PIDVelocityController<LinearVelocity>;
using AngularPIDVelocityController = PIDVelocityController<AngularVelocity>;

// simple vel controller
using LinearSimpleVelocityController = SimpleVelocityController<LinearVelocity>;
using AngularSimpleVelocityController =
  SimpleVelocityController<AngularVelocity>;

template<typename Controller>
    requires Feedforward<Controller, DifferentialSpeeds, LeftRightVoltages>
struct VelocityFeedforward : virtual ControllerBase {
  public:
    Controller velocity_feedforward;

    VelocityFeedforward(Controller velocity_feedforward_controller)
        : velocity_feedforward(velocity_feedforward_controller) {}

    // creates a copy of the controller with different linear feedback
    // controller
    void set_velocity_feedforward(Controller new_velocity_feedforward) {
        this->velocity_feedforward = new_velocity_feedforward;
    }
};

template<typename Controller>
concept hasVelocityFeedforward =
  requires(Controller controller) { controller.velocity_feedforward; };

template<typename Controller>
    requires Feedback<Controller, DifferentialSpeeds, LeftRightVoltages>
struct VelocityFeedback : virtual ControllerBase {
  public:
    Controller velocity_feedback;

    VelocityFeedback(Controller velocity_feedforward_controller)
        : velocity_feedback(velocity_feedforward_controller) {}

    // creates a copy of the controller with different linear feedback
    // controller
    void set_velocity_feedback(Controller new_velocity_feedback) {
        this->velocity_feedback = new_velocity_feedback;
    }
};

template<typename Controller>
concept hasVelocityFeedback =
  requires(Controller controller) { controller.velocity_feedback; };

} // namespace lyfast
} // namespace blazing
