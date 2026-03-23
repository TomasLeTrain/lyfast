#pragma once

#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "lyfast/filters/velocity_estimator.hpp"
#include "lyfast/utils/timestamped_types.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <chrono>
#include <cstdint>
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

    struct target_t {
        std::variant<Voltage, AngularVelocity> target;
        uint32_t timestamp;
    };

  private:
    VelocityEstimator<AngularVelocity>* m_filter;
    AngularSimpleVelocityController m_controller;

    target_t m_target = { 0_volt, 0 };
    Voltage m_commanded_voltage = 0_volt;

  public:
    void resetController() {
        std::lock_guard lock(m_mutex);
        m_controller.reset();
    }

    void update() {
        std::lock_guard lock(m_mutex);

        if (std::holds_alternative<Voltage>(m_target.target)) {
            m_commanded_voltage = std::get<Voltage>(m_target.target);
        } else if (std::holds_alternative<AngularVelocity>(m_target.target)) {
            // before updating controller update with measurement
            m_controller.setLatestMeasurement(
              getTimestampedEstimatedSpeed().velocity,
              getTimestampedEstimatedSpeed().timestamp);

            m_commanded_voltage = m_controller.update();
        }
    }

    void addTarget(std::variant<Voltage, AngularVelocity> new_target,
                   uint32_t timestamp) {
        std::lock_guard lock(m_mutex);

        // if they differ in the type they hold
        // and new type is velocity controlled
        if (new_target.index() != m_target.target.index() &&
            std::holds_alternative<AngularVelocity>(new_target)) {
            // resets controller
            m_controller.reset();
        }

        // if new target is velocity controlled
        if (std::holds_alternative<AngularVelocity>(new_target)) {
            // add target to controller
            auto speed_target = std::get<AngularVelocity>(m_target.target);
            m_controller.addTarget(speed_target, timestamp);
        }

        // update target
        m_target = { new_target, timestamp };
    }

    TimestampedVelocity<AngularVelocity> getTimestampedEstimatedSpeed() {
        // no mutex since it does not interact with this object directly (filter
        // assumed to be thread safe)
        return m_filter->getPredictedState();
    }

    AngularVelocity getEstimatedSpeed() {
        // no mutex since it does not interact with this object directly (filter
        // assumed to be thread safe)
        return getTimestampedEstimatedSpeed().velocity;
    }

    Voltage getCommandedVoltage() {
        std::lock_guard lock(m_mutex);
        return m_commanded_voltage;
    }

    AngularMotorGroupVelocityPlant(VelocityEstimator<AngularVelocity>* filter,
                                   AngularSimpleVelocityController controller)
        : m_filter(filter),
          m_controller(controller) {}
};

// moves controller using velocity estimates from the filters
// plant is not responsible for updating the filters
class LinearMotorGroupVelocityPlant {
  protected:
    pros::Mutex m_mutex;

    struct target_t {
        std::variant<Voltage, LinearVelocity> target;
        uint32_t timestamp;
    };

  private:
    VelocityEstimator<AngularVelocity>* m_filter;
    LinearSimpleVelocityController m_controller;
    Length m_wheel_diameter;

    target_t m_target = { 0_volt, 0 };
    Voltage m_commanded_voltage = 0_volt;

  public:
    void resetController() {
        std::lock_guard lock(m_mutex);
        m_controller.reset();
    }

    void update() {
        std::lock_guard lock(m_mutex);

        if (std::holds_alternative<Voltage>(m_target.target)) {
            m_commanded_voltage = std::get<Voltage>(m_target.target);
        } else if (std::holds_alternative<LinearVelocity>(m_target.target)) {
            // before updating controller update with measurement
            m_controller.setLatestMeasurement(
              getTimestampedEstimatedSpeed().velocity,
              getTimestampedEstimatedSpeed().timestamp);

            m_commanded_voltage = m_controller.update();
        }
    }

    void addTarget(std::variant<Voltage, LinearVelocity> new_target,
                   uint32_t timestamp) {
        std::lock_guard lock(m_mutex);

        // if they differ in the type they hold
        // and new type is velocity controlled
        if (new_target.index() != m_target.target.index() &&
            std::holds_alternative<LinearVelocity>(new_target)) {
            // resets controller
            m_controller.reset();
        }

        // if new target is velocity controlled
        if (std::holds_alternative<LinearVelocity>(new_target)) {
            // add target to controller
            m_controller.addTarget(std::get<LinearVelocity>(m_target.target),
                                   timestamp);
        }

        // update target
        m_target = { new_target, timestamp };
    }

    TimestampedVelocity<LinearVelocity> getTimestampedEstimatedSpeed() {
        // no mutex since it does not interact with this object directly (filter
        // assumed to be thread safe)
        return { toLinear(m_filter->getPredictedState().velocity,
                          m_wheel_diameter),
                 m_filter->getPredictedState().timestamp };
    }

    LinearVelocity getEstimatedSpeed() {
        return getTimestampedEstimatedSpeed().velocity;
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
          m_wheel_diameter(wheel_diameter) {}
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
    LeftRightVoltages m_commanded_voltages { 0_volt, 0_volt };

  public:
    std::optional<TimestampedVelocity<LeftRightSpeeds>> m_measurement =
      std::nullopt;

    // TODO: done temporarily for testing
    void setMeasurement(
      std::optional<TimestampedVelocity<LeftRightSpeeds>> measurement) {
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

    void update() {
        std::lock_guard lock(m_mutex);

        if (std::holds_alternative<LeftRightVoltages>(m_target)) {
            m_commanded_voltages = std::get<LeftRightVoltages>(m_target);
        } else if (std::holds_alternative<DifferentialSpeeds>(m_target)) {
            // before updating controller update with measurement
            m_controller.setLatestMeasurement(
              getTimestampedEstimatedSpeeds().velocity,
              getTimestampedEstimatedSpeeds().timestamp);

            m_commanded_voltages = m_controller.update();
        }
    }

    void
    addTarget(std::variant<LeftRightVoltages, DifferentialSpeeds> new_target,
              uint32_t timestamp) {
        std::lock_guard lock(m_mutex);

        // if they differ in the type they hold
        // and new type is velocity controlled
        if (new_target.index() != m_target.index() &&
            std::holds_alternative<DifferentialSpeeds>(new_target)) {
            // reset controller
            m_controller.reset();
        }

        if (std::holds_alternative<DifferentialSpeeds>(new_target)) {
            m_controller.addTarget(std::get<DifferentialSpeeds>(m_target),
                                   timestamp);
        }

        // update target
        m_target = new_target;
    }

    // TODO: kind of misleading, should eventually remove?
    std::variant<LeftRightVoltages, DifferentialSpeeds> getTarget() {
        std::lock_guard lock(m_mutex);
        return m_target;
    }

    TimestampedVelocity<LeftRightSpeeds> getTimestampedEstimatedSpeeds() {
        if (m_measurement.has_value()) return m_measurement.value();

        // no mutex since it does not interact with this object directly (filter
        // assumed to be thread safe)
        return {
            { toLinear(m_left_filter->getPredictedState().velocity,
             m_wheel_diameter),
             toLinear(m_right_filter->getPredictedState().velocity,
             m_wheel_diameter) },
            // TODO: assumes left and right filter were updated at the same
            // time!
            m_right_filter->getPredictedState().timestamp
        };
    }

    LeftRightSpeeds getEstimatedSpeeds() {
        return getTimestampedEstimatedSpeeds().velocity;
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
          m_wheel_diameter(wheel_diameter) {}
};
} // namespace lyfast
} // namespace blazing
