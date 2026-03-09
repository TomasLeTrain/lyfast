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

  public:
    void resetController() {
        m_controller.reset();
    }

    AngularVelocity getEstimatedSpeed() {
        return m_filter->getPredictedState();
    }

    Voltage controllerUpdate(AngularVelocity target, Time duration) {
        return m_controller.update(getEstimatedSpeed(), target, duration);
    }
};

// moves controller using velocity estimates from the filters
// plant is not responsible for updating the filters
class LinearMotorGroupVelocityPlant {
    EMAVelocityFilter* m_filter;
    LinearSimpleVelocityController m_controller;

    Length m_wheel_diameter;

  public:
    void resetController() {
        m_controller.reset();
    }

    LinearVelocity getEstimatedSpeed() {
        return toLinear(m_filter->getPredictedState(), m_wheel_diameter);
    }

    Voltage controllerUpdate(LinearVelocity target, Time duration) {
        return m_controller.update(getEstimatedSpeed(), target, duration);
    }

    LinearMotorGroupVelocityPlant(
      EMAVelocityFilter* filter,
      SimpleVelocityController<LinearVelocity> controller,
      Length wheel_diameter)
        : m_filter(filter),
          m_controller(controller),
          m_wheel_diameter(wheel_diameter) {}
};

// controls drivetrain with two modes: voltage and velocity.
// velocity gets handled by the given controller

// TODO: mutex
class DrivetrainVelocityPlant {
    pros::MotorGroup* left_motors;
    pros::MotorGroup* right_motors;

    EMAVelocityFilter* m_left_filter;
    EMAVelocityFilter* m_right_filter;
    DifferentialVelocityController m_controller;

    Length m_wheel_diameter;

    FLeftRightVoltages m_commanded_voltages { 0_volt, 0_volt };

    std::variant<LeftRightVoltages, DifferentialSpeeds> m_target;

    void updateMotors(Voltage left_voltage, Voltage right_voltage) {
        m_commanded_voltages =
          FLeftRightVoltages { .left_voltage = left_voltage,
                               .right_voltage = right_voltage };
        left_motors->move_voltage(12 * to_mvolt(left_voltage));
        right_motors->move_voltage(12 * to_mvolt(right_voltage));
    }

  public:
    std::array<Voltage, 2> getCommandedVoltages() {
        return std::array<Voltage, 2> { m_commanded_voltages.left_voltage,
                                        m_commanded_voltages.right_voltage };
    }

    void resetController() {
        m_controller.reset();
    }

    LeftRightSpeeds getEstimatedSpeeds() {
        return {
            toLinear(m_left_filter->getPredictedState(), m_wheel_diameter),
            toLinear(m_right_filter->getPredictedState(), m_wheel_diameter),
        };
    }

    // LeftRightVoltages controllerUpdate(DifferentialSpeeds target,
    //                                    Time duration) {
    //     return m_controller.update(getEstimatedSpeeds(), target, duration);
    // }

    void update(Time dt) {
        if (std::holds_alternative<LeftRightVoltages>(m_target)) {
            auto target = std::get<LeftRightVoltages>(m_target);
        } else {
            return m_controller.update(getEstimatedSpeeds(), target, duration);
            auto target = std::get<DifferentialSpeeds>(m_target);
        }
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
