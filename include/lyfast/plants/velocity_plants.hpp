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
    EMAVelocityFilter* m_filter;
    AngularSimpleVelocityController m_controller;

    std::variant<Voltage, AngularVelocity> m_target;
    Voltage m_commanded_voltage;

    uint32_t m_last_update_timestamp;

    Voltage controllerUpdate(AngularVelocity target, Time duration) {
        return m_controller.update(getEstimatedSpeed(), target, duration);
    }

  public:
    void resetController() {
        std::lock_guard lock(m_mutex);
        m_controller.reset();
        m_last_update_timestamp = pros::millis();
    }

    void update(Time dt) {
        std::lock_guard lock(m_mutex);
        if (std::holds_alternative<Voltage>(m_target)) {
            auto voltage_target = std::get<Voltage>(m_target);
            m_commanded_voltage = voltage_target;
        } else {
            auto speed_target = std::get<AngularVelocity>(m_target);
            auto voltage_target = controllerUpdate(speed_target, dt);

            m_commanded_voltage = voltage_target;
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
        // if they differ in the type they hold
        if (new_target.index() != m_target.index() &&
            std::holds_alternative<AngularVelocity>(new_target)) {
            // resets controller if we go from voltage to velocity
            m_controller.reset();
            m_last_update_timestamp = pros::millis();
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

    AngularMotorGroupVelocityPlant(EMAVelocityFilter* filter,
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
    EMAVelocityFilter* m_filter;
    LinearSimpleVelocityController m_controller;

    Length m_wheel_diameter;

    std::variant<Voltage, LinearVelocity> m_target;
    Voltage m_commanded_voltage;

    uint32_t m_last_update_timestamp;

    Voltage controllerUpdate(LinearVelocity target, Time duration) {
        return m_controller.update(getEstimatedSpeed(), target, duration);
    }

  public:
    void resetController() {
        std::lock_guard lock(m_mutex);
        m_controller.reset();
        m_last_update_timestamp = pros::millis();
    }

    void update(Time dt) {
        std::lock_guard lock(m_mutex);
        if (std::holds_alternative<Voltage>(m_target)) {
            auto voltage_target = std::get<Voltage>(m_target);
            m_commanded_voltage = voltage_target;
        } else {
            auto speed_target = std::get<LinearVelocity>(m_target);
            auto voltage_target = controllerUpdate(speed_target, dt);

            m_commanded_voltage = voltage_target;
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
        // if they differ in the type they hold
        if (new_target.index() != m_target.index() &&
            std::holds_alternative<LinearVelocity>(new_target)) {
            // resets controller if we go from voltage to velocity
            m_controller.reset();
            m_last_update_timestamp = pros::millis();
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

    LinearMotorGroupVelocityPlant(EMAVelocityFilter* filter,
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
    EMAVelocityFilter* m_left_filter;
    EMAVelocityFilter* m_right_filter;
    DifferentialVelocityController m_controller;

    Length m_wheel_diameter;

    std::variant<LeftRightVoltages, DifferentialSpeeds> m_target;
    LeftRightVoltages m_commanded_voltages;

    LeftRightVoltages controllerUpdate(DifferentialSpeeds target,
                                       Time duration) {
        return m_controller.update(getEstimatedSpeeds(), target, duration);
    }

    uint32_t m_last_update_timestamp;

  public:
    const DifferentialVelocityController& getController() {
        return m_controller;
    }

    void resetController() {
        std::lock_guard lock(m_mutex);
        m_controller.reset();
        m_last_update_timestamp = pros::millis();
    }

    void update(Time dt) {
        std::lock_guard lock(m_mutex);
        if (std::holds_alternative<LeftRightVoltages>(m_target)) {
            auto voltage_target = std::get<LeftRightVoltages>(m_target);
            m_commanded_voltages = voltage_target;
        } else {
            auto speed_target = std::get<DifferentialSpeeds>(m_target);
            auto voltage_target = controllerUpdate(speed_target, dt);

            m_commanded_voltages = voltage_target;
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
    setTarget(std::variant<LeftRightVoltages, DifferentialSpeeds> new_target) {
        std::lock_guard lock(m_mutex);
        // if they differ in the type they hold
        if (new_target.index() != m_target.index() &&
            std::holds_alternative<DifferentialSpeeds>(new_target)) {
            // resets controller if we go from voltage to velocity
            m_controller.reset();
            m_last_update_timestamp = pros::millis();
        }

        // update target
        m_target = new_target;
    }

    std::variant<LeftRightVoltages, DifferentialSpeeds> getTarget() {
        std::lock_guard lock(m_mutex);
        return m_target;
    }

    LeftRightSpeeds getEstimatedSpeeds() {
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

    DrivetrainVelocityPlant(EMAVelocityFilter* left_filter,
                            EMAVelocityFilter* right_filter,
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
