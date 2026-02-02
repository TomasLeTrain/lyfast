#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers/clamp.hpp"
#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/drivetrains/drivetrain.hpp"
#include "blazing/tolerances.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <concepts>
#include <functional>
#include <optional>
#include <vector>

namespace blazing {

// used to make the motion changer methods more readable

#define motionChangerMsg                                                  \
    [[nodiscard("motion won't be executed unless an executor is used!")]]

#define motionChanger                                                    \
    [[nodiscard(                                                         \
      "motion won't be executed unless an executor is used!")]] Derived&

#define motionChangerT                                                   \
    template<typename T>                                                 \
    [[nodiscard(                                                         \
      "motion won't be executed unless an executor is used!")]] Derived&

#define motionChangerTU                                                  \
    template<typename T, typename U>                                     \
    [[nodiscard(                                                         \
      "motion won't be executed unless an executor is used!")]] Derived&

#define DerivedReturnType static_cast<Derived&>(*this)
#define ThisDerived       static_cast<Derived*>(this)

struct motionExecutionResult {
    std::optional<bool> inLargeTolerance = std::nullopt;
    std::optional<bool> inSmallTolerance = std::nullopt;
    std::optional<bool> inChainTolerance = std::nullopt;
    bool finished = false;
};

// untemplated class to allow pointers
class MotionBase {
  public:
    virtual void start_motion_callback() {}

    virtual void end_motion_callback() {}

    virtual int getLoopDelayTime() = 0;
    virtual std::optional<motionExecutionResult> execute() = 0;

    // functions meant to be used for chaining motions
    virtual bool setEnabledDrivetrain(bool enabled) {
        return false;
    }

    virtual std::optional<std::vector<Voltage>> getVoltagesDrivetrain() {
        return std::nullopt;
    }

    virtual bool moveVoltagesDrivetrain(std::vector<Voltage> voltages) {
        return false;
    }

    virtual std::optional<Time> getChainTime() {
        return std::nullopt;
    }

    virtual ~MotionBase() = default;
};

template<typename ControllersType,
         typename DrivetrainType,
         typename TrackerType,
         typename TolerancesType,
         typename Derived>
    requires std::derived_from<TolerancesType, TolerancesGroup>
class Motion : public MotionBase {
  public:
    using controllersType = ControllersType;
    using drivetrainType = DrivetrainType;
    using trackerType = TrackerType;
    using tolerancesType = TolerancesType;

  public:
    // these are assumed to have no issues being copied
    ControllersType controllers;
    TolerancesType tolerances;

    // these are taken by reference
    TrackerType& tracker;
    DrivetrainType& drivetrain;

  protected:
    std::optional<Time> chain_time = std::nullopt;

    bool before_motion_func_blocking = false;
    std::function<void()> before_motion_func;
    std::function<void()> after_motion_func;

  public:
    Motion(ControllersType controllers,
           Chassis<DrivetrainType, TrackerType, TolerancesType> chassis)
        : controllers(controllers),
          tolerances(chassis.tolerances),
          tracker(chassis.tracker),
          drivetrain(chassis.drivetrain) {}

    // no copiable
    Motion(const Motion&) = delete;
    Motion& operator=(const Motion&) = delete;

    // movable
    Motion(Motion&&) noexcept = default;
    Motion& operator=(Motion&&) noexcept = default;

    // attempt to override chain functions
    bool setEnabledDrivetrain(bool enabled) override {
        if constexpr (MotionChainableDrivetrain<DrivetrainType>) {
            drivetrain.setEnabled(enabled);
            return true;
        }
        return false;
    };

    std::optional<std::vector<Voltage>> getVoltagesDrivetrain() override {
        if constexpr (MotionChainableDrivetrain<DrivetrainType>) {
            return drivetrain.getVoltages();
        }
        return std::nullopt;
    };

    bool moveVoltagesDrivetrain(std::vector<Voltage> voltages) override {
        if constexpr (MotionChainableDrivetrain<DrivetrainType>) {
            drivetrain.moveVoltages(voltages);
            return true;
        }
        return false;
    };

    std::optional<Time> getChainTime() override {
        return chain_time;
    };

    motionChanger setChainTime(Time chain_time) {
        this->chain_time = chain_time;
        return DerivedReturnType;
    };

    // tracker
    motionChanger executeBeforeMotion(std::function<void()> func,
                                      bool blocking = false) {
        this->before_motion_func = func;
        this->before_motion_func_blocking = blocking;
        return DerivedReturnType;
    }

    motionChanger executeAfterMotion(std::function<void()> func) {
        this->after_motion_func = func;
        return DerivedReturnType;
    }

    // not really relevant to how its supposed to be used
    //
    // motionChanger chainLinearToleranceDuration(
    //                                            Time duration) {
    //     this->tolerances.chain_linear.setDuration(duration);
    //     return DerivedReturnType;
    // }
    //
    // motionChanger chainAngularToleranceDuration(
    //                                             Time duration) {
    //     this->tolerances.chain_angular.setDuration(duration);
    //     return DerivedReturnType;
    // }

    // half circle tolerances
    motionChanger halfcircleTolerance(std::optional<Length> back_tolerance,
                                      Length radius_tolerance = 5_in) {
        this->tolerances.linear.setHalfcircleTolerance(back_tolerance,
                                                       radius_tolerance);
        return DerivedReturnType;
    }

    motionChanger largeHalfcircleTolerance(std::optional<Length> back_tolerance,
                                           Length radius_tolerance = 5_in) {
        this->tolerances.large_linear.setHalfcircleTolerance(back_tolerance,
                                                             radius_tolerance);
        return DerivedReturnType;
    }

    motionChanger chainHalfcircleTolerance(std::optional<Length> back_tolerance,
                                           Length radius_tolerance = 5_in) {

        this->tolerances.chain_linear.setHalfcircleTolerance(back_tolerance,
                                                             radius_tolerance);
        return DerivedReturnType;
    }

    // required to be the same type as original controller
    motionChangerT withLinearFeedbackController(T new_controllers) {
        this->controllers.set_linear_feedback(new_controllers);
        return DerivedReturnType;
    }

    // required to be the same type as original controller
    motionChangerT withAngularFeedbackController(T new_controllers) {
        this->controllers.set_angular_feedback(new_controllers);
        return DerivedReturnType;
    }

    // required to be the same type as original controller
    motionChangerT withLinearVelocityFeedbackController(T new_controllers) {
        this->controllers.set_linear_velocity_feedback(new_controllers);
        return DerivedReturnType;
    }

    // required to be the same type as original controller
    motionChangerT withAngularVelocityFeedbackController(T new_controllers) {
        this->controllers.set_angular_velocity_feedback(new_controllers);
        return DerivedReturnType;
    }

    void start_motion_callback() override {
        // before motion should be blocking - prereq to the motion executing

        if (before_motion_func_blocking) {
            if (before_motion_func) before_motion_func();
        } else {
            pros::Task::create(
              [before_motion_func = this->before_motion_func] {
                  if (before_motion_func) before_motion_func();
              },
              "end motion task");
        }
    }

    void end_motion_callback() override {
        // run it on a separate task - take function by copy
        pros::Task::create(
          [after_motion_func = this->after_motion_func] {
              if (after_motion_func) after_motion_func();
          },
          "end motion task");
    }
};

// allows motions to specify if they use angular/linear components to only show
template<typename Derived>
class AngularMotion {
  public:
    // angular pid changers
    motionChangerT turn_kp(T kp)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDAngularController>
    {
        ThisDerived->controllers.angular_feedback.set_kp(kp);
        return DerivedReturnType;
    }

    motionChangerT turn_ki(T ki)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDAngularController>
    {
        ThisDerived->controllers.angular_feedback.set_ki(ki);
        return DerivedReturnType;
    }

    motionChangerT turn_kd(T kd)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDAngularController>
    {
        ThisDerived->controllers.angular_feedback.set_kd(kd);
        return DerivedReturnType;
    }

    motionChangerT turn_windupRange(T windupRange)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDAngularController>
    {
        ThisDerived->controllers.angular_feedback.set_windupRange(windupRange);
        return DerivedReturnType;
    }

    motionChangerT turn_PIDmaxVolt(T maxVoltage)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDAngularController>
    {
        ThisDerived->controllers.angular_feedback.set_maxOutput(maxVoltage);
        return DerivedReturnType;
    }

    // angular voltage constraints
    motionChangerTU turn_minMaxVolt(T minVoltage, U maxVoltage)
        requires std::derived_from<typename Derived::controllersType,
                                   AngularVoltageClampController>
    {
        ThisDerived->controllers.angular_voltage_clamp.setMin(minVoltage);
        ThisDerived->controllers.angular_voltage_clamp.setMax(maxVoltage);
        return DerivedReturnType;
    }

    motionChangerT turn_minVolt(T minVoltage)
        requires std::derived_from<typename Derived::controllersType,
                                   AngularVoltageClampController>
    {
        ThisDerived->controllers.angular_voltage_clamp.setMin(minVoltage);
        return DerivedReturnType;
    }

    motionChangerT turn_maxVolt(T maxVoltage)
        requires std::derived_from<typename Derived::controllersType,
                                   AngularVoltageClampController>
    {
        ThisDerived->controllers.angular_voltage_clamp.setMax(maxVoltage);
        return DerivedReturnType;
    }

    // angular slew changers
    motionChangerT turn_slew(AngularSlewController new_slew)
        requires hasAngularSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_slew = new_slew;
        return DerivedReturnType;
    }

    motionChangerT turn_accelSlew(T accelSlew)
        requires hasAngularSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_slew.set_accel(accelSlew);
        return DerivedReturnType;
    }

    motionChangerT turn_backwardsAccelSlew(T backwardsAccelSlew)
        requires hasAngularSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_slew.set_backwards_accel(
          backwardsAccelSlew);
        return DerivedReturnType;
    }

    motionChangerT turn_decelSlew(T decelSlew)
        requires hasAngularSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_slew.set_decel(decelSlew);
        return DerivedReturnType;
    }

    motionChangerT turn_backwardsDecelSlew(T backwardsDecelSlew)
        requires hasAngularSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_slew.set_backwards_decel(
          backwardsDecelSlew);
        return DerivedReturnType;
    }

    // tolerance changers
    motionChanger turn_toleranceDuration(Time duration) {
        ThisDerived->tolerances.angular.setDuration(duration);
        return DerivedReturnType;
    }

    motionChanger turn_largeToleranceDuration(Time duration) {
        ThisDerived->tolerances.large_angular.setDuration(duration);
        return DerivedReturnType;
    }

    // Error tolerance changers
    motionChanger turn_errorTolerance(Angle tolerance) {
        ThisDerived->tolerances.angular.setErrorTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger turn_largeErrorTolerance(Angle tolerance) {
        ThisDerived->tolerances.large_angular.setErrorTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger turn_chainErrorTolerance(Angle tolerance) {
        ThisDerived->tolerances.chain_angular.setErrorTolerance(tolerance);
        return DerivedReturnType;
    }

    // velocity tolerance changers
    motionChanger turn_velocityTolerance(AngularVelocity tolerance) {
        ThisDerived->tolerances.angular.setVelocityTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger turn_largeVelocityTolerance(AngularVelocity tolerance) {
        ThisDerived->tolerances.large_angular.setVelocityTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger turn_chainVelocityTolerance(AngularVelocity tolerance) {
        ThisDerived->tolerances.chain_angular.setVelocityTolerance(tolerance);
        return DerivedReturnType;
    }

    // velocity pid changers
    motionChangerT turn_vel_kp(T kp) {
        ThisDerived->controllers.angular_velocity_feedback.set_kp(kp);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_ki(T ki) {
        ThisDerived->controllers.angular_velocity_feedback.set_ki(ki);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_kd(T kd) {
        ThisDerived->controllers.angular_velocity_feedback.set_kd(kd);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_windupRange(T windupRange) {
        ThisDerived->controllers.angular_velocity_feedback.set_windupRange(
          windupRange);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_PIDmaxVel(T maxVoltage) {
        ThisDerived->controllers.angular_velocity_feedback.set_maxOutput(
          maxVoltage);
        return DerivedReturnType;
    }

    // Angular Velocity constraints
    motionChangerTU turn_vel_minMaxVel(T minVelocity, U maxVelocity)
        requires std::derived_from<typename Derived::controllersType,
                                   AngularVelocityClampController>
    {
        ThisDerived->controllers.angular_velocity_clamp.setMin(maxVelocity);
        ThisDerived->controllers.angular_velocity_clamp.setMax(maxVelocity);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_minVel(T minVelocity)
        requires std::derived_from<typename Derived::controllersType,
                                   AngularVelocityClampController>
    {
        ThisDerived->controllers.angular_velocity_clamp.setMin(minVelocity);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_maxVel(T maxVelocity)
        requires std::derived_from<typename Derived::controllersType,
                                   AngularVelocityClampController>
    {
        ThisDerived->controllers.angular_velocity_clamp.setMax(maxVelocity);
        return DerivedReturnType;
    }

    // Angular slew changers
    motionChangerT turn_vel_slew(AngularSlewController new_slew)
        requires hasAngularVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_velocity_slew = new_slew;
        return DerivedReturnType;
    }

    motionChangerT turn_vel_accelSlew(T accelSlew)
        requires hasAngularVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_velocity_slew.set_accel(accelSlew);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_backwardsAccelSlew(T backwardsAccelSlew)
        requires hasAngularVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_velocity_slew.set_backwards_accel(
          backwardsAccelSlew);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_backwardsDecelSlew(T backwardsDecelSlew)
        requires hasAngularVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_velocity_slew.set_backwards_decel(
          backwardsDecelSlew);
        return DerivedReturnType;
    }

    motionChangerT turn_vel_decelSlew(T decelSlew)
        requires hasAngularVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.angular_velocity_slew.set_decel(decelSlew);
        return DerivedReturnType;
    }
};

template<typename Derived>
class LinearMotion {
  public:
    // linear pid changers
    motionChangerT drive_kp(T kp)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDLinearController>
    {
        ThisDerived->controllers.linear_feedback.set_kp(kp);
        return DerivedReturnType;
    }

    motionChangerT drive_ki(T ki)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDLinearController>
    {
        ThisDerived->controllers.linear_feedback.set_ki(ki);
        return DerivedReturnType;
    }

    motionChangerT drive_kd(T kd)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDLinearController>
    {
        ThisDerived->controllers.linear_feedback.set_kd(kd);

        return DerivedReturnType;
    }

    motionChangerT drive_windupRange(T windupRange)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDLinearController>
    {
        ThisDerived->controllers.linear_feedback.set_windupRange(windupRange);
        return DerivedReturnType;
    }

    motionChangerT drive_PIDmaxVolt(T maxVoltage)
        requires std::derived_from<typename Derived::controllersType,
                                   PIDLinearController>
    {
        ThisDerived->controllers.linear_feedback.set_maxOutput(maxVoltage);
        return DerivedReturnType;
    }

    // linear voltage constraints
    motionChangerTU drive_minMaxVolt(T minVoltage, U maxVoltage)
        requires std::derived_from<typename Derived::controllersType,
                                   LinearVoltageClampController>
    {
        ThisDerived->controllers.linear_voltage_clamp.setMin(minVoltage);
        ThisDerived->controllers.linear_voltage_clamp.setMax(maxVoltage);
        return DerivedReturnType;
    }

    motionChangerT drive_minVolt(T minVoltage)
        requires std::derived_from<typename Derived::controllersType,
                                   LinearVoltageClampController>
    {
        ThisDerived->controllers.linear_voltage_clamp.setMin(minVoltage);
        return DerivedReturnType;
    }

    motionChangerT drive_maxVolt(T maxVoltage)
        requires std::derived_from<typename Derived::controllersType,
                                   LinearVoltageClampController>
    {
        ThisDerived->controllers.linear_voltage_clamp.setMax(maxVoltage);
        return DerivedReturnType;
    }

    // linear slew changers
    motionChangerT drive_slew(LinearSlewController new_slew)
        requires hasLinearSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_slew = new_slew;
        return DerivedReturnType;
    }

    motionChangerT drive_accelSlew(T accelSlew)
        requires hasLinearSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_slew.set_accel(accelSlew);
        return DerivedReturnType;
    }

    motionChangerT drive_backwardsAccelSlew(T backwardsAccelSlew)
        requires hasLinearSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_slew.set_backwards_accel(
          backwardsAccelSlew);
        return DerivedReturnType;
    }

    motionChangerT drive_backwardsDecelSlew(T backwardsDecelSlew)
        requires hasLinearSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_slew.set_backwards_decel(
          backwardsDecelSlew);
        return DerivedReturnType;
    }

    motionChangerT drive_decelSlew(T decelSlew)
        requires hasLinearSlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_slew.set_decel(decelSlew);
        return DerivedReturnType;
    }

    // tolerance changers
    motionChanger drive_toleranceDuration(Time duration) {
        ThisDerived->tolerances.linear.setDuration(duration);
        return DerivedReturnType;
    }

    motionChanger drive_largeToleranceDuration(Time duration) {
        ThisDerived->tolerances.large_linear.setDuration(duration);
        return DerivedReturnType;
    }

    motionChanger drive_errorTolerance(Length tolerance) {
        ThisDerived->tolerances.linear.setErrorTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger drive_largeErrorTolerance(Length tolerance) {
        ThisDerived->tolerances.large_linear.setErrorTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger drive_chainErrorTolerance(Length tolerance) {
        ThisDerived->tolerances.chain_linear.setErrorTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger drive_velocityTolerance(LinearVelocity tolerance) {
        ThisDerived->tolerances.linear.setVelocityTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger drive_largeVelocityTolerance(LinearVelocity tolerance) {
        ThisDerived->tolerances.large_linear.setVelocityTolerance(tolerance);
        return DerivedReturnType;
    }

    motionChanger drive_chainVelocityTolerance(LinearVelocity tolerance) {
        ThisDerived->tolerances.change_linear.setVelocityTolerance(tolerance);
        return DerivedReturnType;
    }

    // velocity pid changers
    motionChangerT drive_vel_kp(T kp) {
        ThisDerived->controllers.linear_velocity_feedback.set_kp(kp);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_ki(T ki) {
        ThisDerived->controllers.linear_velocity_feedback.set_ki(ki);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_kd(T kd) {
        ThisDerived->controllers.linear_velocity_feedback.set_kd(kd);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_windupRange(T windupRange) {
        ThisDerived->controllers.linear_velocity_feedback.set_windupRange(
          windupRange);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_PIDmaxVolt(T maxVoltage) {
        ThisDerived->controllers.linear_velocity_feedback.set_maxOutput(
          maxVoltage);
        return DerivedReturnType;
    }

    // linear Velocity constraints
    motionChangerTU drive_vel_minMaxVolt(T minVelocity, U maxVelocity)
        requires std::derived_from<typename Derived::controllersType,
                                   LinearVelocityClampController>
    {
        ThisDerived->controllers.linear_velocity_clamp.setMin(maxVelocity);
        ThisDerived->controllers.linear_velocity_clamp.setMax(maxVelocity);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_minVolt(T minVelocity)
        requires std::derived_from<typename Derived::controllersType,
                                   LinearVelocityClampController>
    {
        ThisDerived->controllers.linear_velocity_clamp.setMin(minVelocity);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_maxVolt(T maxVelocity)
        requires std::derived_from<typename Derived::controllersType,
                                   LinearVelocityClampController>
    {
        ThisDerived->controllers.linear_velocity_clamp.setMax(maxVelocity);
        return DerivedReturnType;
    }

    // linear slew changers
    motionChangerT drive_vel_slew(LinearSlewController new_slew)
        requires hasLinearVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_velocity_slew = new_slew;
        return DerivedReturnType;
    }

    motionChangerT drive_vel_accelSlew(T accelSlew)
        requires hasLinearVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_velocity_slew.set_accel(accelSlew);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_backwardsAccelSlew(T backwardsAccelSlew)
        requires hasLinearVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_velocity_slew.set_backwards_accel(
          backwardsAccelSlew);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_backwardsDecelSlew(T backwardsDecelSlew)
        requires hasLinearVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_velocity_slew.set_backwards_decel(
          backwardsDecelSlew);
        return DerivedReturnType;
    }

    motionChangerT drive_vel_decelSlew(T decelSlew)
        requires hasLinearVelocitySlew<typename Derived::controllersType>
    {
        ThisDerived->controllers.linear_velocity_slew.set_decel(decelSlew);
        return DerivedReturnType;
    }
};

} // namespace blazing
