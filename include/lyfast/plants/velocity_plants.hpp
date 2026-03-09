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

// moves controller using velocity estimates from the filters
// plant is not responsible for updating the filters
class DrivetrainVelocityPlant {
    EMAVelocityFilter* m_left_filter;
    EMAVelocityFilter* m_right_filter;
    DifferentialVelocityController m_controller;

    Length m_wheel_diameter;

  public:
    void resetController() {
        m_controller.reset();
    }

    LeftRightSpeeds getEstimatedSpeeds() {
        return {
            toLinear(m_left_filter->getPredictedState(), m_wheel_diameter),
            toLinear(m_right_filter->getPredictedState(), m_wheel_diameter),
        };
    }

    LeftRightVoltages controllerUpdate(DifferentialSpeeds target,
                                       Time duration) {
        return m_controller.update(getEstimatedSpeeds(), target, duration);
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
