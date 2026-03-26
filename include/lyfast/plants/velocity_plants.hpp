#pragma once

#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "lyfast/filters/ema_filter.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <chrono>
#include <map>
#include <mutex>
#include <variant>

namespace blazing {
namespace lyfast {
// moves controller using velocity estimates from the filters
// plant is not responsible for updating the filters
class AngularMotorGroupVelocityPlant {
  protected:
    pros::Mutex m_mutex;

  private:
    VelocityEstimator<AngularVelocity>* m_filter;
    AngularSimpleVelocityController m_controller;

    std::variant<Voltage, AngularVelocity> m_target = 0_volt;
    Voltage m_commanded_voltage;

    uint32_t m_last_update_timestamp;

    Voltage controllerUpdate(Time duration) {
        return m_controller.update(getEstimatedSpeed(), duration);
    }

  public:
    void resetController() {
        std::lock_guard lock(m_mutex);

        m_controller.reset();
    }

    void update(Time dt) {
        std::lock_guard lock(m_mutex);

        if (std::holds_alternative<Voltage>(m_target)) {
            m_commanded_voltage = std::get<Voltage>(m_target);
        } else if (std::holds_alternative<AngularVelocity>(m_target)) {
            m_commanded_voltage = controllerUpdate(dt);
        }

        m_last_update_timestamp = pros::millis();
    }

    void updateToTimestamp(uint32_t timestamp) {
        // can't go back in time
        if (timestamp < m_last_update_timestamp) return;

        // if both are equal then still update, let the controller handle it
        update(from_msec(timestamp - m_last_update_timestamp));
    }

    void setTarget(std::variant<Voltage, AngularVelocity> new_target) {
        std::lock_guard lock(m_mutex);

        bool new_target_is_vel =
          std::holds_alternative<AngularVelocity>(new_target);

        // if they differ in the type they hold
        if (new_target.index() != m_target.index() && new_target_is_vel) {
            // resets controller if we go from voltage to velocity
            m_controller.reset();
        }

        // update target for controller, if being used
        if (new_target_is_vel) {
            m_controller.setTarget(std::get<AngularVelocity>(new_target));
        }

        // update target
        m_target = new_target;
    }

    AngularVelocity getEstimatedSpeed() {
        // no mutex since it does not interact with this object directly (filter
        // assumed to be thread safe)
        return m_filter->getPredictedState();
    }

    Voltage getCommandedVoltage() {
        std::lock_guard lock(m_mutex);
        return m_commanded_voltage;
    }

    AngularMotorGroupVelocityPlant(VelocityEstimator<AngularVelocity>* filter,
                                   AngularSimpleVelocityController controller)
        : m_filter(filter),
          m_controller(controller),
          m_last_update_timestamp(pros::millis()) {}
};

// moves controller using velocity estimates from the filters
// plant is not responsible for updating the filters
class LinearMotorGroupVelocityPlant {
  protected:
    pros::Mutex m_mutex;

  private:
    VelocityEstimator<AngularVelocity>* m_filter;
    LinearSimpleVelocityController m_controller;

    Length m_wheel_diameter;

    std::variant<Voltage, LinearVelocity> m_target = 0_volt;
    Voltage m_commanded_voltage;

    uint32_t m_last_update_timestamp;

    Voltage controllerUpdate(Time duration) {
        return m_controller.update(getEstimatedSpeed(), duration);
    }

  public:
    void resetController() {
        std::lock_guard lock(m_mutex);
        m_controller.reset();
    }

    void update(Time dt) {
        std::lock_guard lock(m_mutex);
        if (std::holds_alternative<Voltage>(m_target)) {
            m_commanded_voltage = std::get<Voltage>(m_target);
        } else if (std::holds_alternative<LinearVelocity>(m_target)) {
            m_commanded_voltage = controllerUpdate(dt);
        }
        m_last_update_timestamp = pros::millis();
    }

    void updateToTimestamp(uint32_t timestamp) {
        // can't go back in time
        if (timestamp < m_last_update_timestamp) return;

        // if both are equal then still update, let the controller handle it
        update(from_msec(timestamp - m_last_update_timestamp));
    }

    void setTarget(std::variant<Voltage, LinearVelocity> new_target) {
        std::lock_guard lock(m_mutex);

        bool new_target_is_vel =
          std::holds_alternative<LinearVelocity>(new_target);

        // if they differ in the type they hold
        if (new_target.index() != m_target.index() && new_target_is_vel) {
            // resets controller if we go from voltage to velocity
            m_controller.reset();
        }

        // update target for controller, if being used
        if (new_target_is_vel) {
            m_controller.setTarget(std::get<LinearVelocity>(new_target));
        }

        // update target
        m_target = new_target;
    }

    LinearVelocity getEstimatedSpeed() {
        // no mutex since it does not interact with this object directly (filter
        // assumed to be thread safe)
        return toLinear(m_filter->getPredictedState(), m_wheel_diameter);
    }

    Voltage getCommandedVoltage() {
        std::lock_guard lock(m_mutex);
        return m_commanded_voltage;
    }

    LinearMotorGroupVelocityPlant(VelocityEstimator<AngularVelocity>* filter,
                                  LinearSimpleVelocityController controller,
                                  Length wheel_diameter)
        : m_filter(filter),
          m_controller(controller),
          m_wheel_diameter(wheel_diameter),
          m_last_update_timestamp(pros::millis()) {}
};

// controls drivetrain with two modes: voltage and velocity.
// velocity gets handled by the given controller

// TODO: mutex
class DrivetrainVelocityPlant {
  protected:
    pros::Mutex m_mutex;

  private:
    VelocityEstimator<AngularVelocity>* m_left_filter;
    VelocityEstimator<AngularVelocity>* m_right_filter;
    DifferentialVelocityController m_controller;

    Length m_wheel_diameter;

    std::variant<LeftRightVoltages, DifferentialSpeeds> m_target =
      LeftRightVoltages { 0_volt, 0_volt };
    LeftRightVoltages m_commanded_voltages;

    uint32_t m_last_update_timestamp;

    LeftRightVoltages controllerUpdate(Time duration) {
        return m_controller.update(getEstimatedSpeeds(), duration);
    }

  public:
    std::optional<LeftRightSpeeds> m_measurement = std::nullopt;

    void setMeasurement(std::optional<LeftRightSpeeds> measurement) {
        m_measurement = measurement;
    }

  public:
    const DifferentialVelocityController& getController() {
        return m_controller;
    }

    void resetController() {
        std::lock_guard lock(m_mutex);
        m_controller.reset();
    }

    void update(Time dt) {
        std::lock_guard lock(m_mutex);

        if (std::holds_alternative<LeftRightVoltages>(m_target)) {
            m_commanded_voltages = std::get<LeftRightVoltages>(m_target);
        } else if (std::holds_alternative<DifferentialSpeeds>(m_target)) {
            m_commanded_voltages = controllerUpdate(dt);
        }

        m_last_update_timestamp = pros::millis();
    }

    void updateToTimestamp(uint32_t timestamp) {
        // can't go back in time
        if (timestamp < m_last_update_timestamp) return;

        // if both are equal then still update, let the controller handle it
        update(from_msec(timestamp - m_last_update_timestamp));
    }

    void
    setTarget(std::variant<LeftRightVoltages, DifferentialSpeeds> new_target,
              uint32_t timestamp) {
        std::lock_guard lock(m_mutex);

        bool new_target_is_vel =
          std::holds_alternative<DifferentialSpeeds>(new_target);

        // if they differ in the type they hold
        if (new_target.index() != m_target.index() && new_target_is_vel) {
            // resets controller if we go from voltage to velocity
            m_controller.reset();
        }

        // update target for controller, if being used
        if (new_target_is_vel) {
            m_controller.setTarget(std::get<DifferentialSpeeds>(new_target),
                                   timestamp);
        }

        // update target
        m_target = new_target;
    }

    void
    setTarget(std::variant<LeftRightVoltages, DifferentialSpeeds> new_target) {
        setTarget(new_target, pros::millis());
    }

    std::variant<LeftRightVoltages, DifferentialSpeeds> getTarget() {
        std::lock_guard lock(m_mutex);
        return m_target;
    }

    LeftRightSpeeds getEstimatedSpeeds() {
        if (m_measurement.has_value()) return m_measurement.value();
        // no mutex since it does not interact with this object directly (filter
        // assumed to be thread safe)
        return {
            toLinear(m_left_filter->getPredictedState(), m_wheel_diameter),
            toLinear(m_right_filter->getPredictedState(), m_wheel_diameter),
        };
    }

    LeftRightVoltages getCommandedVoltages() {
        std::lock_guard lock(m_mutex);
        return m_commanded_voltages;
    }

    DrivetrainVelocityPlant(VelocityEstimator<AngularVelocity>* left_filter,
                            VelocityEstimator<AngularVelocity>* right_filter,
                            DifferentialVelocityController controller,
                            Length wheel_diameter)
        : m_left_filter(left_filter),
          m_right_filter(right_filter),
          m_controller(controller),
          m_wheel_diameter(wheel_diameter),
          m_last_update_timestamp(pros::millis()) {}
};
} // namespace lyfast
} // namespace blazing
