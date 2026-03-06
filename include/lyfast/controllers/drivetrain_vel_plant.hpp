#pragma once

#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <map>

namespace blazing {
namespace lyfast {

// returns -1 if gearing is invalid
// TODO: move to blazing utils?
inline AngularVelocity gearingToVelocity(pros::MotorGears gearing) {
    if (gearing == pros::MotorGears::rpm_600)
        return 600_rpm;
    else if (gearing == pros::MotorGears::rpm_200)
        return 200_rpm;
    else if (gearing == pros::MotorGears::rpm_100)
        return 100_rpm;
    // if encoder units are not set then it defaults to 200?
    return 200_rpm;
}

class MotorGroupKalmanFilter {
  public:
    using CovarianceUnit = Exponentiated<AngularVelocity, std::ratio<2>>;

    struct Input {
        Voltage voltage;
    };

    struct State {
        AngularVelocity velocity;
    };

    struct Constants {
        // the supposed final rpm after applying gearing. For example:
        // 600 rpm goes through 2:1 gear ratio, now becomes 1200 rpm (final
        // gearing rpm)
        AngularVelocity final_gearing_rpm;

        // kv constant of the motor
        Divided<Voltage, AngularVelocity> Kv;
        // ka constant of the motor
        Divided<Voltage, AngularAcceleration> Ka;
        // ks constant of the motor
        Voltage Ks;

        CovarianceUnit process_covariance = units::square(5_rpm);

        // cov = factor * measurement^2
        // covariance increases on larger velocities
        float measurement_covariance_factor = units::square(0.4);
        // minimum covariance
        CovarianceUnit measurement_covariance_offset = units::square(5_rpm);
    };

  private:
    pros::MotorGroup* motor_group;

    Constants m_constants;
    State m_state_estimate;
    Input m_input;
    CovarianceUnit m_covariance { 0 };

    void correctSingleMotor(std::uint8_t idx) {
        AngularVelocity motor_reported_velocity =
          AngularVelocity(motor_group->get_actual_velocity(idx));
        pros::MotorGears gearing = motor_group->get_gearing(idx);

        AngularVelocity geared_motor_reported_velocity =
          (motor_reported_velocity / gearingToVelocity(gearing)) *
          m_constants.final_gearing_rpm;

        std::cout << "motor vel: " << geared_motor_reported_velocity.convert(radps)
                  << std::endl;

        // TODO: perform basic filtering on the reported velocity?

        AngularVelocity innovation =
          geared_motor_reported_velocity - m_state_estimate.velocity;
        std::cout << "innovation: " << innovation.internal() << std::endl;

        CovarianceUnit measurement_covariance =
          m_constants.measurement_covariance_factor *
            units::square(geared_motor_reported_velocity) +
          m_constants.measurement_covariance_offset;
        std::cout << "meas cov: " << measurement_covariance.internal()
                  << std::endl;

        CovarianceUnit innovation_covariance =
          m_covariance + measurement_covariance;
        std::cout << "innovation cov: " << innovation_covariance.internal()
                  << std::endl;

        float gain = m_covariance / innovation_covariance;
        std::cout << "gain: " << gain << std::endl;

        m_state_estimate.velocity =
          m_state_estimate.velocity + gain * innovation;
        std::cout << "new vel: " << m_state_estimate.velocity << std::endl;

        // update covariance from measurement
        m_covariance = (1 - gain) * m_covariance;
        std::cout << "new cov: " << m_covariance << std::endl;
        std::cout << "ENDED" << std::endl;
    }

    // update input (voltage) based on motor data
    void updateInput() {
        m_input.voltage = 0_volt;
        int motor_count = 0;
        for (auto voltage : motor_group->get_voltage_all()) {
            m_input.voltage += from_mvolt(voltage) / 12.0;
            motor_count++;
        }
        m_input.voltage /= static_cast<float>(motor_count);
        std::cout << "input updated to " << m_input.voltage << std::endl;
    }

  public:
    // take measurements from the motor(s) and correct model based on them
    void correct() {
        // apply correction for each motor
        std::cout << "correcting" << std::endl;
        for (int i = 0; i < motor_group->size(); i++) {
            correctSingleMotor(i);
        }
    }

    // predict x_k+1 from x_k and u_k
    void predict(Time dt) {
        Voltage V_eff;
        if (m_input.voltage < m_constants.Ks) {
            // voltage low enough that it cannot overcome friction, resulting in
            // no movement
            V_eff = 0_volt;
        } else {
            V_eff =
              m_input.voltage - m_constants.Ks * units::sgn(m_input.voltage);
        }
        std::cout << "START" << std::endl;
        std::cout << "V_eff " << V_eff << std::endl;

        // V = kv * v + ka * a + ks * sgn(V)
        // [V - sgn(v) * ks] = kv * v + ka * a
        //
        // V_eff = [V - sgn(v) * ks]
        //
        // V_eff = kv * v + ka * a
        //
        // a = (V_eff - kv * v) / ka
        // a = V_eff * 1/ka - (kv/ka) * v
        // v = v + a * t
        // v = v + (V_eff * 1/ka - (kv/ka) * v) * dt
        // v = v(1 - (kv/ka) * dt) + V_eff * (1/ka * dt)
        //
        // A = 1 - (kv/ka) * dt
        // B = 1/ka * dt
        //
        // v = A * v + B * V_eff

        double A = 1 - (m_constants.Kv / m_constants.Ka) * dt;
        auto B = (1.0 / m_constants.Ka) * dt;
        std::cout << "A/B " << A << " " << B << std::endl;

        AngularVelocity new_predicted_v =
          m_state_estimate.velocity * A + B * V_eff;

        std::cout << "new predicted v " << new_predicted_v << std::endl;

        m_state_estimate = { .velocity = new_predicted_v };

        m_covariance = A * A * m_covariance + m_constants.process_covariance;
        std::cout << "new predict covairance " << m_covariance << std::endl;

        // update input after prediction
        std::cout << "updating input " << std::endl;
        updateInput();
    }

    Input getInput() {
        return m_input;
    }

    State getPredictedState() {
        return m_state_estimate;
    }

    void setPredictedState(State new_state, CovarianceUnit covariance) {
        m_state_estimate = new_state;
        m_covariance = covariance;
    }

    MotorGroupKalmanFilter(pros::MotorGroup* motors,
                           Constants constants,
                           State initial_state_estimate,
                           CovarianceUnit initial_covariance)
        : motor_group(motors),
          m_constants(constants),
          m_state_estimate(initial_state_estimate),
          m_input({ .voltage = 0_volt }),
          m_covariance(initial_covariance) {}
};

class AngularMotorGroupVelocityPlant {
    MotorGroupKalmanFilter m_filter;
    SimpleVelocityController<AngularVelocity> m_controller;

  public:
    void reset() {
        MotorGroupKalmanFilter::State state { .velocity = 0_radps };
        // TODO: determine default?
        MotorGroupKalmanFilter::CovarianceUnit covariance { units::square(
          600_rpm) };

        m_filter.setPredictedState(state, covariance);
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
        MotorGroupKalmanFilter::State state { .velocity = 0_radps };
        // TODO: determine default?
        MotorGroupKalmanFilter::CovarianceUnit covariance { units::square(
          600_rpm) };

        m_filter.setPredictedState(state, covariance);
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
        MotorGroupKalmanFilter::State state { .velocity = 0_radps };
        // TODO: determine default?
        MotorGroupKalmanFilter::CovarianceUnit covariance { units::square(
          600_rpm) };

        m_left_filter.setPredictedState(state, covariance);
        m_right_filter.setPredictedState(state, covariance);
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
