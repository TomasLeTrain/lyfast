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
#include <queue>

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

// only lowers linear velocity to stop saturation
DifferentialSpeeds
desaturatePrioritizeAngularDiffSpeeds(DifferentialSpeeds target,
                                      Length track_width,
                                      LinearVelocity max_velocity);

// normal desaturation of velocities
DifferentialSpeeds desaturateDifferentialSpeeds(DifferentialSpeeds target,
                                                Length track_width,
                                                LinearVelocity max_velocity);

template<typename VelUnit>
struct FeedforwardVelocityControllerParams {
    KvUnits<VelUnit> Kv;
    KaUnits<VelUnit> Ka;
    Voltage Ks;
    Time Ka_delta_time;
    VelUnit low_target_threshold { 0 };
};

template<typename VelUnit>
struct PIDVelocityControllerParams {
    KvUnits<VelUnit> Kp { 0 };
    KvUnits<VelUnit> Kp_close { 0 };
    KvUnits<VelUnit> Kp_low { 0 };
    VelUnit low_threshold { 0 };
    VelUnit close_threshold { 0 };
    Divided<Voltage, Multiplied<VelUnit, Time>> Ki { 0 };
    std::optional<VelUnit> Ki_windup = std::nullopt;

    Voltage max_output { 1_volt };
    double tbh_factor { 0.0 };
};

template<typename VelUnit>
class FeedforwardVelocityController {
  private:
    using AccelT = Divided<VelUnit, Time>;
    FeedforwardVelocityControllerParams<VelUnit> m_params;

    VelUnit m_target_velocity { 0 };
    AccelT m_target_acceleration { 0 };

  public:
    void setTarget(VelUnit target_velocity, AccelT target_acceleration) {
        m_target_velocity = target_velocity;
        m_target_acceleration = target_acceleration;
    }

    void setTarget(VelUnit target_velocity) {
        setTarget(target_velocity,
                  (target_velocity - m_target_velocity) /
                    m_params.Ka_delta_time);
    }

    Voltage updateKvKa() {
        // target low enough that accel is basically instant
        if (units::abs(m_target_velocity) < m_params.low_target_threshold) {
            m_target_acceleration = Divided<VelUnit, Time> { 0 };
        }

        Voltage result { // kv
                         m_target_velocity * m_params.Kv +
                         // ka
                         m_target_acceleration * m_params.Ka
        };

        return result;
    }

    // allows applying ks later in the chain if other processes are done in
    // between
    Voltage applyKs(Voltage output) {
        // apply ks at the end
        return output + units::sgn(output) * m_params.Ks;
    }

    Voltage update() {
        return applyKs(updateKvKa());
    }

    void reset() {
        m_target_velocity = VelUnit { 0 };
        m_target_acceleration = AccelT { 0 };
    }

    std::pair<VelUnit, AccelT> getTarget() {
        return { m_target_velocity, m_target_acceleration };
    }

    FeedforwardVelocityControllerParams<VelUnit> getParams() {
        return m_params;
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

    // local variables, members so they can be accessed between multiple methods
    Multiplied<VelUnit, Time> current_integral { 0 };
    VelUnit error;

    VelUnit m_target_velocity { 0 };

    std::queue<VelUnit> m_targets;
    int num_targets = 5;

  public:
    void setTarget(VelUnit target_velocity) {
        // m_target_velocity = target_velocity;
        m_targets.push(target_velocity);
        if (m_targets.size() > num_targets) m_targets.pop();
        // 1. 60 msec  -  0 msec   -> went from 1 to two items
        // 2. 80 msec  -  20 msec  -> went frmo 2 to 3 items
        // 3. 100 msec  -  40 msec -> went frmo 3 to 4 items
        // 4. 120 msec  -  60 msec -> went frmo 4 items to 5 items
        // 5. 140 msec  -  80 msec -> went frmo 5 items to 6 items, start
        // popping
    }

    Voltage unclampedUpdate(VelUnit measurement, Time duration) {
        // const VelUnit curr_target = m_target_velocity;
        const VelUnit curr_target = m_targets.front();

        error = curr_target - measurement;

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

        auto curr_Kp = m_params.Kp;

        // if the error is high then normal kp still applies
        if (units::abs(m_target_velocity) <= m_params.low_threshold &&
            units::abs(error) <= m_params.low_threshold) {
            curr_Kp = m_params.Kp_low;
        } else if (units::abs(error) < m_params.close_threshold) {
            curr_Kp = m_params.Kp_close;
        }

        // dont use integral if outside ki windup range
        if (m_params.Ki_windup.has_value() &&
            m_params.Ki_windup.value() < units::abs(error)) {
            current_integral = Multiplied<VelUnit, Time> { 0 };
        }

        Voltage result {
            // kp
            curr_Kp * error +
              // ki
              // TODO: temporary testing of kv integrator
              m_params.Ki * current_integral * m_target_velocity.internal(),
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

    Voltage update(VelUnit measurement, Time duration) {
        return compensateForSaturation(unclampedUpdate(measurement, duration));
    }

    void reset() {
        integral = Multiplied<VelUnit, Time> { 0 };
        m_target_velocity = VelUnit { 0 };
        m_targets.push(VelUnit { 0 });
        last_error = std::nullopt;
    }

    VelUnit getTarget() {
        return m_target_velocity;
    }

    PIDVelocityControllerParams<VelUnit> getParams() {
        return m_params;
    }

    void setParams(PIDVelocityControllerParams<VelUnit> params) {
        m_params = params;
    }

    PIDVelocityController(PIDVelocityControllerParams<VelUnit> params)
        : m_params(params) {
        m_targets.push(VelUnit { 0 });
    }
};

template<typename VelUnit>
class SimpleVelocityController {
    FeedforwardVelocityController<VelUnit> m_feedforward;

    // single pid for both possibilities?
    PIDVelocityController<VelUnit> m_pid;

    VelUnit m_target;

  public:
    void setTarget(VelUnit target) {
        m_feedforward.setTarget(target);
        m_pid.setTarget(target);
        m_target = target;
    }

    Voltage update(VelUnit measurement, Time duration) {
        Voltage u_feedforward = m_feedforward.updateKvKa();
        Voltage u_feedback = m_pid.unclampedUpdate(measurement, duration);

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

    VelUnit getTarget() {
        return m_target;
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
    TargetT m_target;

  public:
    void setTarget(TargetT target) {
        LinearVelocity v_target = target.linear + target.angular;
        m_linear.setTarget(target.linear);
        m_angular.setTarget(target.angular);
        m_pid.setTarget(v_target);
        m_target = target;
    }

    Voltage update(LinearVelocity measurement, Time duration) {
        Voltage u_linear = m_linear.updateKvKa();
        Voltage u_angular = m_angular.updateKvKa();
        Voltage u_feedback = m_pid.unclampedUpdate(measurement, duration);

        Voltage result = u_linear + u_angular + u_feedback;

        // apply ks only from linear output
        result = m_linear.applyKs(result);

        // apply final pid step
        result = m_pid.compensateForSaturation(result);

        return result;
    }

    TargetT getTarget() {
        return m_target;
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

struct FFLeftRightVelocityControllerParams {
    KvUnits<LinearVelocity> left_Kv;
    KaUnits<LinearVelocity> left_Ka;
    KsUnits left_Ks;

    KvUnits<LinearVelocity> right_Kv;
    KaUnits<LinearVelocity> right_Ka;
    KsUnits right_Ks;

    Time Ka_delta_time;
    LinearVelocity low_target_threshold;
};

struct PIDLeftRightVelocityControllerParams {
    KvUnits<LinearVelocity> left_Kp { 0 };
    KvUnits<LinearVelocity> left_Kp_close { 0 };
    KvUnits<LinearVelocity> left_Kp_low { 0 };
    LinearVelocity left_low_threshold { 0 };
    LinearVelocity left_close_threshold { 0 };
    KiUnits<LinearVelocity> left_Ki { 0 };
    std::optional<LinearVelocity> left_Ki_windup = std::nullopt;

    Voltage left_max_output { 1_volt };
    double left_tbh_factor { 0.0 };

    KvUnits<LinearVelocity> right_Kp { 0 };
    KvUnits<LinearVelocity> right_Kp_close { 0 };
    KvUnits<LinearVelocity> right_Kp_low { 0 };
    LinearVelocity right_low_threshold { 0 };
    LinearVelocity right_close_threshold { 0 };
    KiUnits<LinearVelocity> right_Ki { 0 };
    std::optional<LinearVelocity> right_Ki_windup = std::nullopt;

    Voltage right_max_output { 1_volt };
    double right_tbh_factor { 0.0 };
};

// makes it easier to construct entire drivetrain controller without
// constructing all the objects first
struct DifferentialVelocityControllerParams {
    FFLeftRightVelocityControllerParams linear;
    FFLeftRightVelocityControllerParams angular;
    PIDLeftRightVelocityControllerParams pid;

    DrivetrainSideVelocityController constructLeftController() {
        return { FeedforwardVelocityController<LinearVelocity>({
                   .Kv = linear.left_Kv,
                   .Ka = linear.left_Ka,
                   .Ks = linear.left_Ks,
                   .Ka_delta_time = linear.Ka_delta_time,
                   .low_target_threshold = linear.low_target_threshold,
                 }),
                 FeedforwardVelocityController<LinearVelocity>({
                   .Kv = angular.left_Kv,
                   .Ka = angular.left_Ka,
                   .Ks = angular.left_Ks,
                   .Ka_delta_time = angular.Ka_delta_time,
                   .low_target_threshold = angular.low_target_threshold,
                 }),
                 PIDVelocityController<LinearVelocity>({
                   .Kp = pid.left_Kp,
                   .Kp_close = pid.left_Kp_close,
                   .Kp_low = pid.left_Kp_low,
                   .low_threshold = pid.left_low_threshold,
                   .close_threshold = pid.left_close_threshold,
                   .Ki = pid.left_Ki,
                   .Ki_windup = pid.left_Ki_windup,
                   .max_output = pid.left_max_output,
                   .tbh_factor = pid.left_tbh_factor,
                 }) };
    }

    DrivetrainSideVelocityController constructRightController() {
        return { FeedforwardVelocityController<LinearVelocity>({
                   .Kv = linear.right_Kv,
                   .Ka = linear.right_Ka,
                   .Ks = linear.right_Ks,
                   .Ka_delta_time = linear.Ka_delta_time,
                   .low_target_threshold = linear.low_target_threshold,
                 }),
                 FeedforwardVelocityController<LinearVelocity>({
                   .Kv = angular.right_Kv,
                   .Ka = angular.right_Ka,
                   .Ks = angular.right_Ks,
                   .Ka_delta_time = angular.Ka_delta_time,
                   .low_target_threshold = angular.low_target_threshold,
                 }),
                 PIDVelocityController<LinearVelocity>({
                   .Kp = pid.right_Kp,
                   .Kp_close = pid.right_Kp_close,
                   .Kp_low = pid.right_Kp_low,
                   .low_threshold = pid.right_low_threshold,
                   .close_threshold = pid.right_close_threshold,
                   .Ki = pid.right_Ki,
                   .Ki_windup = pid.right_Ki_windup,
                   .max_output = pid.right_max_output,
                   .tbh_factor = pid.right_tbh_factor,
                 }) };
    }
};

class DifferentialVelocityController {
    DrivetrainSideVelocityController m_left_controller;
    DrivetrainSideVelocityController m_right_controller;

    LinearVelocity m_max_velocity;
    Length m_track_width;
    bool m_prioritize_angular = false;

    DifferentialSpeeds m_target;

  public:
    void setTarget(DifferentialSpeeds target) {
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
        m_target = target;

        LinearVelocity target_linear_velocity = target.linear_velocity;
        // angular velocity converted to linear velocity wheel speeds
        LinearVelocity converted_angular_velocity =
          (target.angular_velocity / rad) * track_radius;

        m_left_controller.setTarget({ .linear = target_linear_velocity,
                                      .angular = -converted_angular_velocity });

        m_right_controller.setTarget({ .linear = target_linear_velocity,
                                       .angular = converted_angular_velocity });
    }

    LeftRightVoltages update(LeftRightSpeeds measurement, Time duration) {
        Voltage left_voltage =
          m_left_controller.update(measurement.left_vel, duration);
        Voltage right_voltage =
          m_right_controller.update(measurement.right_vel, duration);

        LeftRightVoltages result = { left_voltage, right_voltage };

        return result;
    }

    void reset() {
        m_left_controller.reset();
        m_right_controller.reset();
    }

    DifferentialSpeeds getTarget() {
        return m_target;
    }

    const DrivetrainSideVelocityController& getLeftController() {
        return m_left_controller;
    }

    const DrivetrainSideVelocityController& getRightController() {
        return m_right_controller;
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
      bool prioritize_angular)
        : m_left_controller(left_controller),
          m_right_controller(right_controller),
          m_max_velocity(max_velocity),
          m_track_width(track_width),
          m_prioritize_angular(prioritize_angular) {}

    DifferentialVelocityController(
      DifferentialVelocityControllerParams controller_params,
      LinearVelocity max_velocity,
      Length track_width,
      bool prioritize_angular)
        : m_left_controller(controller_params.constructLeftController()),
          m_right_controller(controller_params.constructRightController()),
          m_max_velocity(max_velocity),
          m_track_width(track_width),
          m_prioritize_angular(prioritize_angular) {}
};

// explicit declarations
// feedforward
// extern template struct FeedforwardVelocityControllerParams<LinearVelocity>;
// extern template struct FeedforwardVelocityControllerParams<AngularVelocity>;
//
// extern template class FeedforwardVelocityController<LinearVelocity>;
// extern template class FeedforwardVelocityController<AngularVelocity>;
//
// // PID
// extern template struct PIDVelocityControllerParams<LinearVelocity>;
// extern template struct PIDVelocityControllerParams<AngularVelocity>;
//
// extern template class PIDVelocityController<LinearVelocity>;
// extern template class PIDVelocityController<AngularVelocity>;
//
// // simple vel controller
// extern template class SimpleVelocityController<LinearVelocity>;
// extern template class SimpleVelocityController<AngularVelocity>;

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
