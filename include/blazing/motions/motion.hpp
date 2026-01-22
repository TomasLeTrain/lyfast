#pragma once

#include "blazing/chassis.hpp"
#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/controllers/voltage_clamp.hpp"
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
#define motionChanger                                                          \
    template<typename Self>                                                    \
    [[nodiscard("motion won't be executed unless an executor is used!")]] auto

#define motionChangerT                                                         \
    template<typename Self, typename T>                                        \
    [[nodiscard("motion won't be executed unless an executor is used!")]] auto

#define motionChangerTU                                                        \
    template<typename Self, typename T, typename U>                            \
    [[nodiscard("motion won't be executed unless an executor is used!")]] auto

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
         typename TolerancesType>
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

    motionChanger setChainTime(this Self&& self, Time chain_time) {
        self.chain_time = chain_time;
        return self.getReference();
    };

    // tracker
    motionChanger executeBeforeMotion(this Self&& self,
                                      std::function<void()> func,
                                      bool blocking = false) {
        self.before_motion_func = func;
        self.before_motion_func_blocking = blocking;
        return self.getReference();
    }

    motionChanger executeAfterMotion(this Self&& self,
                                     std::function<void()> func) {
        self.after_motion_func = func;
        return self.getReference();
    }

    // not really relevant to how its supposed to be used
    //
    // motionChanger chainLinearToleranceDuration(this Self&& self,
    //                                            Time duration) {
    //     self.tolerances.chain_linear.setDuration(duration);
    //     return self.getReference();
    // }
    //
    // motionChanger chainAngularToleranceDuration(this Self&& self,
    //                                             Time duration) {
    //     self.tolerances.chain_angular.setDuration(duration);
    //     return self.getReference();
    // }

    // half circle tolerances
    motionChanger halfcircleTolerance(this Self&& self,
                                      std::optional<Length> back_tolerance,
                                      Length radius_tolerance = 5_in) {
        self.tolerances.linear.setHalfcircleTolerance(back_tolerance,
                                                      radius_tolerance);
        return self.getReference();
    }

    motionChanger largeHalfcircleTolerance(this Self&& self,
                                           std::optional<Length> back_tolerance,
                                           Length radius_tolerance = 5_in) {
        self.tolerances.large_linear.setHalfcircleTolerance(back_tolerance,
                                                            radius_tolerance);
        return self.getReference();
    }

    motionChanger chainHalfcircleTolerance(this Self&& self,
                                           std::optional<Length> back_tolerance,
                                           Length radius_tolerance = 5_in) {

        self.tolerances.chain_linear.setHalfcircleTolerance(back_tolerance,
                                                            radius_tolerance);
        return self.getReference();
    }

    // required to be the same type as original controller
    motionChangerT withLinearFeedbackController(this Self&& self,
                                                T new_controllers) {
        self.controllers.with_linear_feedback(new_controllers);
        return self.getReference();
    }

    // required to be the same type as original controller
    motionChangerT withAngularFeedbackController(this Self&& self,
                                                 T new_controllers) {
        self.controllers.with_angular_feedback(new_controllers);
        return self.getReference();
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
class AngularMotion {
  public:
    // angular pid changers
    motionChangerT turn_kp(this Self&& self, T kp)
        requires std::derived_from<typename Self::controllersType,
                                   PIDAngularController>
    {
        self.controllers.angular_feedback.set_kp(kp);
        return self.getReference();
    }

    motionChangerT turn_ki(this Self&& self, T ki)
        requires std::derived_from<typename Self::controllersType,
                                   PIDAngularController>
    {
        self.controllers.angular_feedback.set_ki(ki);
        return self.getReference();
    }

    motionChangerT turn_kd(this Self&& self, T kd)
        requires std::derived_from<typename Self::controllersType,
                                   PIDAngularController>
    {
        self.controllers.angular_feedback.set_kd(kd);
        return self.getReference();
    }

    motionChangerT turn_windupRange(this Self&& self, T windupRange)
        requires std::derived_from<typename Self::controllersType,
                                   PIDAngularController>
    {
        self.controllers.angular_feedback.set_windupRange(windupRange);
        return self.getReference();
    }

    motionChangerT turn_PIDmaxVolt(this Self&& self, T maxVoltage)
        requires std::derived_from<typename Self::controllersType,
                                   PIDAngularController>
    {
        self.controllers.angular_feedback.set_maxVoltage(maxVoltage);
        return self.getReference();
    }

    // angular voltage constraints
    motionChangerTU
    turn_minMaxVolt(this Self&& self, T minVoltage, U maxVoltage)
        requires std::derived_from<typename Self::controllersType,
                                   AngularVoltageClampController>
    {
        self.controllers.angular_voltage_clamp.setMin(minVoltage);
        self.controllers.angular_voltage_clamp.setMax(maxVoltage);
        return self.getReference();
    }

    motionChangerT turn_minVolt(this Self&& self, T minVoltage)
        requires std::derived_from<typename Self::controllersType,
                                   AngularVoltageClampController>
    {
        self.controllers.angular_voltage_clamp.setMin(minVoltage);
        return self.getReference();
    }

    motionChangerT turn_maxVolt(this Self&& self, T maxVoltage)
        requires std::derived_from<typename Self::controllersType,
                                   AngularVoltageClampController>
    {
        self.controllers.angular_voltage_clamp.setMax(maxVoltage);
        return self.getReference();
    }

    // angular slew changers
    motionChangerT turn_slew(this Self&& self, AngularSlewController new_slew)
        requires hasAngularSlew<typename Self::controllersType>
    {
        self.controllers.angular_slew = new_slew;
        return self.getReference();
    }

    motionChangerT turn_accelSlew(this Self&& self, T accelSlew)
        requires hasAngularSlew<typename Self::controllersType>
    {
        self.controllers.angular_slew.set_accel(accelSlew);
        return self.getReference();
    }

    motionChangerT turn_backwardsAccelSlew(this Self&& self,
                                           T backwardsAccelSlew)
        requires hasAngularSlew<typename Self::controllersType>
    {
        self.controllers.angular_slew.set_backwards_accel(backwardsAccelSlew);
        return self.getReference();
    }

    motionChangerT turn_decelSlew(this Self&& self, T decelSlew)
        requires hasAngularSlew<typename Self::controllersType>
    {
        self.controllers.angular_slew.set_decel(decelSlew);
        return self.getReference();
    }

    motionChangerT turn_backwardsDecelSlew(this Self&& self,
                                           T backwardsDecelSlew)
        requires hasAngularSlew<typename Self::controllersType>
    {
        self.controllers.angular_slew.set_backwards_decel(backwardsDecelSlew);
        return self.getReference();
    }

    // tolerance changers
    motionChanger turn_toleranceDuration(this Self&& self, Time duration) {
        self.tolerances.angular.setDuration(duration);
        return self.getReference();
    }

    motionChanger turn_largeToleranceDuration(this Self&& self, Time duration) {
        self.tolerances.large_angular.setDuration(duration);
        return self.getReference();
    }

    // Error tolerance changers
    motionChanger turn_errorTolerance(this Self&& self, Angle tolerance) {
        self.tolerances.angular.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger turn_largeErrorTolerance(this Self&& self, Angle tolerance) {
        self.tolerances.large_angular.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger turn_chainErrorTolerance(this Self&& self, Angle tolerance) {
        self.tolerances.chain_angular.setErrorTolerance(tolerance);
        return self.getReference();
    }

    // velocity tolerance changers
    motionChanger turn_velocityTolerance(this Self&& self,
                                         AngularVelocity tolerance) {
        self.tolerances.angular.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger turn_largeVelocityTolerance(this Self&& self,
                                              AngularVelocity tolerance) {
        self.tolerances.large_angular.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger turn_chainVelocityTolerance(this Self&& self,
                                              AngularVelocity tolerance) {
        self.tolerances.chain_angular.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    // velocity pid changers
    motionChangerT turn_vel_kp(this Self&& self, T kp) {
        self.controllers.angular_feedback.controller1.set_kp(kp);
        return self.getReference();
    }

    motionChangerT turn_vel_ki(this Self&& self, T ki) {
        self.controllers.angular_feedback.controller1.set_ki(ki);
        return self.getReference();
    }

    motionChangerT turn_vel_kd(this Self&& self, T kd) {
        self.controllers.angular_feedback.controller1.set_kd(kd);
        return self.getReference();
    }

    motionChangerT turn_vel_windupRange(this Self&& self, T windupRange) {
        self.controllers.angular_feedback.controller1.set_windupRange(
          windupRange);
        return self.getReference();
    }

    motionChangerT turn_vel_PIDmaxVolt(this Self&& self, T maxVoltage) {
        self.controllers.angular_feedback.controller1.set_maxVoltage(
          maxVoltage);
        return self.getReference();
    }
};

class LinearMotion {
  public:
    // linear pid changers
    motionChangerT drive_kp(this Self&& self, T kp)
        requires std::derived_from<typename Self::controllersType,
                                   PIDLinearController>
    {
        self.controllers.linear_feedback.set_kp(kp);
        return self.getReference();
    }

    motionChangerT drive_ki(this Self&& self, T ki)
        requires std::derived_from<typename Self::controllersType,
                                   PIDLinearController>
    {
        self.controllers.linear_feedback.set_ki(ki);
        return self.getReference();
    }

    motionChangerT drive_kd(this Self&& self, T kd)
        requires std::derived_from<typename Self::controllersType,
                                   PIDLinearController>
    {
        self.controllers.linear_feedback.set_kd(kd);

        return self.getReference();
    }

    motionChangerT drive_windupRange(this Self&& self, T windupRange)
        requires std::derived_from<typename Self::controllersType,
                                   PIDLinearController>
    {
        self.controllers.linear_feedback.set_windupRange(windupRange);
        return self.getReference();
    }

    motionChangerT drive_PIDmaxVolt(this Self&& self, T maxVoltage)
        requires std::derived_from<typename Self::controllersType,
                                   PIDLinearController>
    {
        self.controllers.linear_feedback.set_maxVoltage(maxVoltage);
        return self.getReference();
    }

    // linear voltage constraints
    motionChangerTU
    drive_minMaxVolt(this Self&& self, T minVoltage, U maxVoltage)
        requires std::derived_from<typename Self::controllersType,
                                   LinearVoltageClampController>
    {
        self.controllers.linear_voltage_clamp.setMin(minVoltage);
        self.controllers.linear_voltage_clamp.setMax(maxVoltage);
        return self.getReference();
    }

    motionChangerT drive_minVolt(this Self&& self, T minVoltage)
        requires std::derived_from<typename Self::controllersType,
                                   LinearVoltageClampController>
    {
        self.controllers.linear_voltage_clamp.setMin(minVoltage);
        return self.getReference();
    }

    motionChangerT drive_maxVolt(this Self&& self, T maxVoltage)
        requires std::derived_from<typename Self::controllersType,
                                   LinearVoltageClampController>
    {
        self.controllers.linear_voltage_clamp.setMax(maxVoltage);
        return self.getReference();
    }

    // linear slew changers
    motionChangerT drive_slew(this Self&& self, LinearSlewController new_slew)
        requires hasLinearSlew<typename Self::controllersType>
    {
        self.controllers.linear_slew = new_slew;
        return self.getReference();
    }

    motionChangerT drive_accelSlew(this Self&& self, T accelSlew)
        requires hasLinearSlew<typename Self::controllersType>
    {
        self.controllers.linear_slew.set_accel(accelSlew);
        return self.getReference();
    }

    motionChangerT drive_backwardsAccelSlew(this Self&& self,
                                            T backwardsAccelSlew)
        requires hasLinearSlew<typename Self::controllersType>
    {
        self.controllers.linear_slew.set_backwards_accel(backwardsAccelSlew);
        return self.getReference();
    }

    motionChangerT drive_backwardsDecelSlew(this Self&& self,
                                            T backwardsDecelSlew)
        requires hasLinearSlew<typename Self::controllersType>
    {
        self.controllers.linear_slew.set_backwards_decel(backwardsDecelSlew);
        return self.getReference();
    }

    motionChangerT drive_decelSlew(this Self&& self, T decelSlew)
        requires hasLinearSlew<typename Self::controllersType>
    {
        self.controllers.linear_slew.set_decel(decelSlew);
        return self.getReference();
    }

    // tolerance changers
    motionChanger drive_toleranceDuration(this Self&& self, Time duration) {
        self.tolerances.linear.setDuration(duration);
        return self.getReference();
    }

    motionChanger drive_largeToleranceDuration(this Self&& self,
                                               Time duration) {
        self.tolerances.large_linear.setDuration(duration);
        return self.getReference();
    }

    motionChanger drive_errorTolerance(this Self&& self, Length tolerance) {
        self.tolerances.linear.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger drive_largeErrorTolerance(this Self&& self,
                                            Length tolerance) {
        self.tolerances.large_linear.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger drive_chainErrorTolerance(this Self&& self,
                                            Length tolerance) {
        self.tolerances.chain_linear.setErrorTolerance(tolerance);
        return self.getReference();
    }

    motionChanger drive_velocityTolerance(this Self&& self,
                                          LinearVelocity tolerance) {
        self.tolerances.linear.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger drive_largeVelocityTolerance(this Self&& self,
                                               LinearVelocity tolerance) {
        self.tolerances.large_linear.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    motionChanger drive_chainVelocityTolerance(this Self&& self,
                                               LinearVelocity tolerance) {
        self.tolerances.change_linear.setVelocityTolerance(tolerance);
        return self.getReference();
    }

    // velocity pid changers
    motionChangerT drive_vel_kp(this Self&& self, T kp) {
        self.controllers.linear_feedback.controller1.set_kp(kp);
        return self.getReference();
    }

    motionChangerT drive_vel_ki(this Self&& self, T ki) {
        self.controllers.linear_feedback.controller1.set_ki(ki);
        return self.getReference();
    }

    motionChangerT drive_vel_kd(this Self&& self, T kd) {
        self.controllers.linear_feedback.controller1.set_kd(kd);
        return self.getReference();
    }

    motionChangerT drive_vel_windupRange(this Self&& self, T windupRange) {
        self.controllers.linear_feedback.controller1.set_windupRange(
          windupRange);
        return self.getReference();
    }

    motionChangerT drive_vel_PIDmaxVolt(this Self&& self, T maxVoltage) {
        self.controllers.linear_feedback.controller1.set_maxVoltage(maxVoltage);
        return self.getReference();
    }
};

} // namespace blazing
