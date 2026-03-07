#pragma once

#include "blazing/utils.hpp"
#include "lyfast/controllers/MotorGroupKalmanFilter.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

class AngularMotorGroupVelocityPlant {
    MotorGroupKalmanFilter m_filter;
    SimpleVelocityController<AngularVelocity> m_controller;

  public:
    void reset() {
        m_filter.reset();
    }

    void updateFilter(Time duration) {
        m_filter.predict(duration);
    }

    AngularVelocity getEstimatedSpeed() {
        return m_filter.getPredictedState().velocity;
    }

    Voltage controllerUpdate(AngularVelocity target, Time duration) {
        return m_controller.update(getEstimatedSpeed(), target, duration);
    }

    AngularMotorGroupVelocityPlant(
      MotorGroupKalmanFilter filter,
      SimpleVelocityController<AngularVelocity> controller)
        : m_filter(filter),
          m_controller(controller) {}
};

class LinearMotorGroupVelocityPlant {
    MotorGroupKalmanFilter m_filter;
    SimpleVelocityController<LinearVelocity> m_controller;

    Length m_wheel_diameter;

  public:
    void resetFilter() {
        m_filter.reset();
    }

    void resetController() {
        m_controller.reset();
    }

    void updateFilter(Time duration) {
        m_filter.predict(duration);
    }

    LinearVelocity getEstimatedSpeed() {
        return toLinear(m_filter.getPredictedState().velocity,
                        m_wheel_diameter);
    }

    Voltage controllerUpdate(LinearVelocity target, Time duration) {
        return m_controller.update(getEstimatedSpeed(), target, duration);
    }

    LinearMotorGroupVelocityPlant(
      MotorGroupKalmanFilter filter,
      SimpleVelocityController<LinearVelocity> controller,
      Length wheel_diameter)
        : m_filter(filter),
          m_controller(controller),
          m_wheel_diameter(wheel_diameter) {}
};

class DrivetrainVelocityPlant {
    MotorGroupKalmanFilter m_left_filter;
    MotorGroupKalmanFilter m_right_filter;
    DifferentialVelocityController m_controller;

    Length m_wheel_diameter;

  public:
    void resetFilter() {
        m_left_filter.reset();
        m_right_filter.reset();
    }

    void resetController() {
        m_controller.reset();
    }

    void updateFilter(Time duration) {
        m_left_filter.predict(duration);
        m_right_filter.predict(duration);
    }

    LeftRightSpeeds getEstimatedSpeeds() {
        return {
            toLinear(m_left_filter.getPredictedState().velocity,
                     m_wheel_diameter),
            toLinear(m_right_filter.getPredictedState().velocity,
                     m_wheel_diameter),
        };
    }

    LeftRightVoltages controllerUpdate(DifferentialSpeeds target,
                                       Time duration) {
        return m_controller.update(getEstimatedSpeeds(), target, duration);
    }

    DrivetrainVelocityPlant(MotorGroupKalmanFilter left_filter,
                            MotorGroupKalmanFilter right_filter,
                            DifferentialVelocityController controller,
                            Length wheel_diameter)
        : m_left_filter(left_filter),
          m_right_filter(right_filter),
          m_controller(controller),
          m_wheel_diameter(wheel_diameter) {}
};
} // namespace lyfast
} // namespace blazing
