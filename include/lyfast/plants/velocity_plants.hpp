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
#include <variant>

namespace blazing {
namespace lyfast {
// moves controller using velocity estimates from the filters
// plant is not responsible for updating the filters
class AngularMotorGroupVelocityPlant {
    EMAVelocityFilter* m_filter;
    AngularSimpleVelocityController m_controller;

    std::variant<Voltage, AngularVelocity> m_target;
    Voltage m_commanded_voltage;

    Voltage controllerUpdate(AngularVelocity target, Time duration) {
        return m_controller.update(getEstimatedSpeed(), target, duration);
    }

  public:
    void resetController() {
        m_controller.reset();
    }

    void update(Time dt) {
        if (std::holds_alternative<Voltage>(m_target)) {
            auto voltage_target = std::get<Voltage>(m_target);
            m_commanded_voltage = voltage_target;
        } else {
            auto speed_target = std::get<AngularVelocity>(m_target);
            auto voltage_target = controllerUpdate(speed_target, dt);

            m_commanded_voltage = voltage_target;
        }
    }

    void setTarget(std::variant<Voltage, AngularVelocity> new_target) {
        // if they differ in the type they hold
        if (new_target.index() != m_target.index() &&
            std::holds_alternative<AngularVelocity>(new_target)) {
            // resets controller if we go from voltage to velocity
            resetController();
        }

        // update target
        m_target = new_target;
    }

    AngularVelocity getEstimatedSpeed() {
        return m_filter->getPredictedState();
    }

    Voltage getCommandedVoltage() {
        return m_commanded_voltage;
    }

    AngularMotorGroupVelocityPlant(EMAVelocityFilter* filter,
                                   AngularSimpleVelocityController controller,
                                   Length wheel_diameter)
        : m_filter(filter),
          m_controller(controller) {}
};

// moves controller using velocity estimates from the filters
// plant is not responsible for updating the filters
class LinearMotorGroupVelocityPlant {
    EMAVelocityFilter* m_filter;
    LinearSimpleVelocityController m_controller;

    Length m_wheel_diameter;

    std::variant<Voltage, LinearVelocity> m_target;
    Voltage m_commanded_voltage;

    Voltage controllerUpdate(LinearVelocity target, Time duration) {
        return m_controller.update(getEstimatedSpeed(), target, duration);
    }

  public:
    void resetController() {
        m_controller.reset();
    }

    void update(Time dt) {
        if (std::holds_alternative<Voltage>(m_target)) {
            auto voltage_target = std::get<Voltage>(m_target);
            m_commanded_voltage = voltage_target;
        } else {
            auto speed_target = std::get<LinearVelocity>(m_target);
            auto voltage_target = controllerUpdate(speed_target, dt);

            m_commanded_voltage = voltage_target;
        }
    }

    void setTarget(std::variant<Voltage, LinearVelocity> new_target) {
        // if they differ in the type they hold
        if (new_target.index() != m_target.index() &&
            std::holds_alternative<LinearVelocity>(new_target)) {
            // resets controller if we go from voltage to velocity
            resetController();
        }

        // update target
        m_target = new_target;
    }

    LinearVelocity getEstimatedSpeed() {
        return toLinear(m_filter->getPredictedState(), m_wheel_diameter);
    }

    Voltage getCommandedVoltage() {
        return m_commanded_voltage;
    }

    LinearMotorGroupVelocityPlant(EMAVelocityFilter* filter,
                                  LinearSimpleVelocityController controller,
                                  Length wheel_diameter)
        : m_filter(filter),
          m_controller(controller),
          m_wheel_diameter(wheel_diameter) {}
};

// controls drivetrain with two modes: voltage and velocity.
// velocity gets handled by the given controller

// TODO: mutex
class DrivetrainVelocityPlant {
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

  public:
    void resetController() {
        m_controller.reset();
    }

    void update(Time dt) {
        if (std::holds_alternative<LeftRightVoltages>(m_target)) {
            auto voltage_target = std::get<LeftRightVoltages>(m_target);
            m_commanded_voltages = voltage_target;
        } else {
            auto speed_target = std::get<DifferentialSpeeds>(m_target);
            auto voltage_target = controllerUpdate(speed_target, dt);

            m_commanded_voltages = voltage_target;
        }
    }

    void
    setTarget(std::variant<LeftRightVoltages, DifferentialSpeeds> new_target) {
        // if they differ in the type they hold
        if (new_target.index() != m_target.index() &&
            std::holds_alternative<DifferentialSpeeds>(new_target)) {
            // resets controller if we go from voltage to velocity
            resetController();
        }

        // update target
        m_target = new_target;
    }

    std::variant<LeftRightVoltages, DifferentialSpeeds> getTarget() {
        return m_target;
    }

    LeftRightSpeeds getEstimatedSpeeds() {
        return {
            toLinear(m_left_filter->getPredictedState(), m_wheel_diameter),
            toLinear(m_right_filter->getPredictedState(), m_wheel_diameter),
        };
    }

    LeftRightVoltages getCommandedVoltages() {
        return m_commanded_voltages;
    }

    DrivetrainVelocityPlant(EMAVelocityFilter* left_filter,
                            EMAVelocityFilter* right_filter,
                            DifferentialVelocityController controller,
                            Length wheel_diameter)
        : m_left_filter(left_filter),
          m_right_filter(right_filter),
          m_controller(controller),
          m_wheel_diameter(wheel_diameter) {}
};
} // namespace lyfast
} // namespace blazing
