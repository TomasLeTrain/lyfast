#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedforward/feedforward.hpp"
#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "lyfast/utils/timestamped_types.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <ios>
#include <queue>

namespace blazing {
namespace lyfast {

// new architecture:
// update() -> calculates outputs from measurements and targets
// addTarget(target, timestamp) -> add desired target at some timestamp
// addMeasurement(target, timestamp) -> add measurement at some timestamp

// new feedforward Concept
template<typename Controller, typename Input, typename Output>
concept NewFeedforward =
  requires(Controller controller, Input target, Time timestamp) {
      { controller.update() } -> std::same_as<Output>;
      { controller.addTarget(target, timestamp) } -> std::same_as<void>;
  };

// new feedback Concept
template<typename Controller, typename Input, typename Output>
concept NewFeedback = requires(Controller controller,
                               Input target,
                               Input measurement,
                               Time timestamp) {
    { controller.update() } -> std::same_as<Output>;
    { controller.addTarget(target, timestamp) } -> std::same_as<void>;
    { controller.addMeasurement(measurement, timestamp) } -> std::same_as<void>;
};

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

DifferentialSpeeds
desaturatePrioritizeAngularDiffSpeeds(DifferentialSpeeds target,
                                      Length track_width,
                                      LinearVelocity max_velocity);

DifferentialSpeeds desaturateDifferentialSpeeds(DifferentialSpeeds target,
                                                Length track_width,
                                                LinearVelocity max_velocity);

template<typename VelUnit>
struct FeedforwardVelocityControllerParams {
    KvUnits<VelUnit> Kv;
    KaUnits<VelUnit> Ka;
    Voltage Ks;
    uint32_t lookahead_time { 0 };
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
    uint32_t lookahead_time { 0 };

    Voltage max_output { 1_volt };
    double tbh_factor { 1.0 };
};

template<typename VelUnit>
class FeedforwardVelocityController {
  public:
    using target_t = TimestampedVelocity<VelUnit>;

  private:
    FeedforwardVelocityControllerParams<VelUnit> m_params;

    std::deque<target_t> m_target_queue;

    // searches for current_target a timestamp >= now + lookahead_time
    // returns [last_target, current_target] or nullopt
    std::optional<std::pair<target_t, target_t>> getCurrentTargets() {
        uint32_t target_time = pros::millis() + m_params.lookahead_time;

        // can't update due to no targets
        if (m_target_queue.empty()) {
            std::cout << "FF: queue is empty!" << std::endl;
            return std::nullopt;
        }

        // pops elements from queue until second element (current target) has
        // timestamp >= target_time
        for (; m_target_queue.size() > 1 &&
               m_target_queue[1].timestamp < target_time;
             m_target_queue.pop_front());

        // either all timestamps are >= or < target_time
        // in > case: use for vel info, regardless of timestamp
        // in <= case: use oldest for vel info, regardless of timestamp
        if (m_target_queue.size() == 1 || // all timestamps < target_time
            m_target_queue.front().timestamp >= target_time) {
            return std::pair { m_target_queue.front(), m_target_queue.front() };
        }

        // get current target
        // measurement from right before current measurement
        return std::pair { m_target_queue.front(), m_target_queue[1] };
    }

  public:
    Voltage updateKvKa() {
        const auto targets = getCurrentTargets();

        // no possible targets to follow
        if (!targets.has_value()) {
            return 0_volt;
        }

        const auto& [last_target, current_target] = targets.value();

        VelUnit delta_velocity = current_target.velocity - last_target.velocity;
        uint32_t discrete_delta_time =
          current_target.timestamp - last_target.timestamp;
        Time delta_time = from_msec(discrete_delta_time);

        // in case delta time is zero assume no accel
        Divided<VelUnit, Time> target_accel = discrete_delta_time == 0 ?
                                                Divided<VelUnit, Time> { 0 } :
                                                delta_velocity / delta_time;

        // target low enough that accel is basically instant
        if (units::abs(current_target.velocity) <
            m_params.low_target_threshold) {
            target_accel = Divided<VelUnit, Time> { 0 };
        }

        Voltage result { // kv
                         current_target.velocity * m_params.Kv +
                         // ka
                         target_accel * m_params.Ka
        };

        return result;
    }

    // allows applying ks later in the chain if other processes are done in
    // between
    Voltage applyKs(Voltage output) {
        // apply ks at the end
        return output + units::sgn(output) * m_params.Ks;
    }

    // NOTE: targets should be queued in chronological order!
    // TODO: could switch to priority queue to support out of order targets
    void addTarget(VelUnit target, uint32_t timestamp) {
        m_target_queue.emplace_back(target, timestamp);
    }

    Voltage update() {
        Voltage result = updateKvKa();
        result = applyKs(result);

        return result;
    }

    FeedforwardVelocityControllerParams<VelUnit> getParams() {
        return m_params;
    }

    void reset() {
        m_target_queue.clear();
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
  public:
    using target_t = TimestampedVelocity<VelUnit>;

  private:
    PIDVelocityControllerParams<VelUnit> m_params;

    Multiplied<VelUnit, Time> m_integral { 0 };

    // local variables, members so they can be accessed between multiple methods
    Multiplied<VelUnit, Time> current_integral { 0 };
    VelUnit error;

    std::queue<target_t> m_target_queue;
    target_t m_latest_measurement;
    std::optional<target_t> last_error = std::nullopt;

    std::optional<target_t> getCurrentTarget() {
        uint32_t target_time = pros::millis() - m_params.lookahead_time;

        // want to find first target with timestamp >= target_time
        for (; m_target_queue.size() > 1 &&
               m_target_queue.front().timestamp < target_time;
             m_target_queue.pop());

        if (m_target_queue.empty()) {
            std::cout << "PID: no queued targets!" << std::endl;
            return std::nullopt;
        }

        // front has timestamp < target_time then no targets match target_time
        // in this case its assumed its a constant command and its still used
        return m_target_queue.front();
    }

  public:
    Voltage unclampedUpdate() {
        std::optional<target_t> curr_target = getCurrentTarget();
        if (!curr_target.has_value()) {
            return 0_volt;
        }

        error = curr_target->velocity - m_latest_measurement.velocity;

        current_integral = m_integral;

        if (last_error)
            // use trapezoidal approximation
            current_integral += (error + last_error->velocity) *
                                from_msec(m_latest_measurement.timestamp -
                                          last_error->timestamp) *
                                0.5;
        // else
        //     // use Riemann sum approximation
        //     current_integral += error * 10_msec;

        // decrease integral by some amount when crossing error to minimize
        // overshooot due to the integral
        if (last_error &&
            units::sgn(error) != units::sgn(last_error->velocity)) {
            current_integral *= m_params.tbh_factor;
            // update integral even if saturating?
            m_integral = current_integral;
        }

        auto curr_Kp = m_params.Kp;

        // if the error is high then normal kp still applies
        if (units::abs(curr_target->velocity) <= m_params.low_threshold &&
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
            // TODO: temporary testing of kv integrator (????)
            m_params.Ki * current_integral * curr_target->velocity.internal()
        };

        last_error = { error, m_latest_measurement.timestamp };

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
            m_integral = current_integral;
        }

        result =
          units::clamp(result, -m_params.max_output, m_params.max_output);

        return result;
    }

    void setLatestMeasurement(VelUnit measurement, uint32_t timestamp) {
        m_latest_measurement = { measurement, timestamp };
    }

    // NOTE: targets should be queued in chronological order!
    void addTarget(VelUnit target, uint32_t timestamp) {
        m_target_queue.emplace(target, timestamp);
    }

    // supported for backwards compatibility
    // Voltage update(VelUnit measurement, VelUnit target, Time duration) {
    //     setLatestMeasurement(measurement, now());
    //     addTarget(target, now() + duration);
    //
    //     Voltage result = unclampedUpdate();
    //     result = compensateForSaturation(result);
    //
    //     return result;
    // }

    void reset() {
        m_integral = Multiplied<VelUnit, Time> { 0 };
        last_error = std::nullopt;
        // clear queue
        m_target_queue = std::queue<target_t>();
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
    void addTarget(VelUnit target, uint32_t timestamp) {
        m_feedforward.addTarget(target, timestamp);
        m_pid.addTarget(target, timestamp);
    }

    void setLatestMeasurement(VelUnit measurement, uint32_t timestamp) {
        m_pid.setLatestMeasurement(measurement, timestamp);
    }

    Voltage update() {
        Voltage u_feedforward = m_feedforward.updateKvKa();
        Voltage u_feedback = m_pid.unclampedUpdate();

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

    FeedforwardVelocityControllerParams<VelUnit> getFeedforwardParams() {
        return m_feedforward.getParams();
    }

    PIDVelocityControllerParams<VelUnit> getFeedbackParams() {
        return m_pid.getParams();
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
    Voltage update() {
        Voltage u_linear = m_linear.updateKvKa();
        Voltage u_angular = m_angular.updateKvKa();
        Voltage u_feedback = m_pid.unclampedUpdate();

        Voltage result = u_linear + u_angular + u_feedback;

        // apply ks only from linear output
        result = m_linear.applyKs(result);

        // apply final pid step
        result = m_pid.compensateForSaturation(result);

        return result;
    }

    void addTarget(TargetT target, uint32_t timestamp) {
        m_linear.addTarget(target.linear, timestamp);
        m_angular.addTarget(target.angular, timestamp);
        m_pid.addTarget(target.linear + target.angular, timestamp);
    }

    void setLatestMeasurement(LinearVelocity measurement, uint32_t timestamp) {
        m_pid.setLatestMeasurement(measurement, timestamp);
    }

    void reset() {
        m_linear.reset();
        m_angular.reset();
        m_pid.reset();
    }

    FeedforwardVelocityControllerParams<LinearVelocity>
    getLinearFeedforwardParams() {
        return m_linear.getParams();
    }

    FeedforwardVelocityControllerParams<LinearVelocity>
    getAngularFeedforwardParams() {
        return m_angular.getParams();
    }

    PIDVelocityControllerParams<LinearVelocity> getFeedbackParams() {
        return m_pid.getParams();
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

    LinearVelocity low_target_threshold;

    uint32_t lookahead_time { 0 };
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

    uint32_t lookahead_time { 0 };
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
                   .lookahead_time = linear.lookahead_time,
                   .low_target_threshold = linear.low_target_threshold,
                 }),
                 FeedforwardVelocityController<LinearVelocity>({
                   .Kv = angular.left_Kv,
                   .Ka = angular.left_Ka,
                   .Ks = angular.left_Ks,
                   .lookahead_time = angular.lookahead_time,
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
                   .lookahead_time = pid.lookahead_time,
                   .max_output = pid.left_max_output,
                   .tbh_factor = pid.left_tbh_factor,
                 }) };
    }

    DrivetrainSideVelocityController constructRightController() {
        return { FeedforwardVelocityController<LinearVelocity>({
                   .Kv = linear.right_Kv,
                   .Ka = linear.right_Ka,
                   .Ks = linear.right_Ks,
                   .lookahead_time = linear.lookahead_time,
                   .low_target_threshold = linear.low_target_threshold,
                 }),
                 FeedforwardVelocityController<LinearVelocity>({
                   .Kv = angular.right_Kv,
                   .Ka = angular.right_Ka,
                   .Ks = angular.right_Ks,
                   .lookahead_time = angular.lookahead_time,
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
                   .lookahead_time = pid.lookahead_time,
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

    // used for getting and filtering velocity
    std::optional<LeftRightSpeeds> last_velocities = std::nullopt;

  public:
    LeftRightVoltages update() {
        return { m_left_controller.update(), m_right_controller.update() };
    }

    void addTarget(DifferentialSpeeds target, uint32_t timestamp) {
        const Length track_radius = m_track_width / 2.0;

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
        LinearVelocity converted_angular_velocity =
          (target.angular_velocity / rad) * track_radius;

        m_left_controller.addTarget({ .linear = target_linear_velocity,
                                      .angular = -converted_angular_velocity },
                                    timestamp);

        m_right_controller.addTarget({ .linear = target_linear_velocity,
                                       .angular = converted_angular_velocity },
                                     timestamp);
    }

    void setLatestMeasurement(LeftRightSpeeds measurement, uint32_t timestamp) {
        m_left_controller.setLatestMeasurement(measurement.left_vel, timestamp);
        m_right_controller.setLatestMeasurement(measurement.right_vel,
                                                timestamp);
    }

    void reset() {
        last_velocities = std::nullopt;
        m_left_controller.reset();
        m_right_controller.reset();
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
