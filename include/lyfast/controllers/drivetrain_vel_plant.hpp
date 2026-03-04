#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <map>

// returns -1 if gearing is invalid
inline AngularVelocity gearingToVelocity(pros::MotorGears gearing) {
    if (gearing == pros::MotorGears::rpm_600)
        return 600_rpm;
    else if (gearing == pros::MotorGears::rpm_200)
        return 200_rpm;
    else if (gearing == pros::MotorGears::rpm_100)
        return 100_rpm;
    // invalid
    return -1_rpm;
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

        CovarianceUnit process_covariance = units::square(10_rpm);

        // cov = factor * measurement^2
        // covariance increases on larger velocities
        float measurement_covariance_factor = 0.01;
        // minimum covariance
        CovarianceUnit measurement_covariance_offset = units::square(5_rpm);
    };

    // stores per-motor state
    // struct MotorState {
    //     std::uint32_t last_timestamp;
    //     std::int32_t last_position;
    // };

    // std::map<std::uint8_t, MotorState> motor_states;

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
        // TODO: perform basic filtering on the reported velocity?

        // std::uint32_t curr_timestamp;
        // std::int32_t raw_position =
        //   motor_group->get_raw_position(&curr_timestamp, idx);
        //
        // // TODO: perform filtering on tick based velocity due to possible
        // // timings?
        //
        // if (motor_states.contains(idx)) {
        //     // we do have previous state for this motor
        //     auto& motor_state = motor_states[idx];
        //
        //     Time position_delta_time =
        //       from_msec((curr_timestamp - motor_state.last_timestamp));
        //     Number position_delta = raw_position - motor_state.last_position;
        //
        //     Frequency tick_velocity = position_delta / position_delta_time;
        //
        //     // TODO: what now?
        //     float ratio = 3600_rpm / constants.final_gearing_rpm;
        //
        //     Divided<Number, Angle> vel_to_ticks = (50 / rot) * ratio;
        //
        //     // TODO: state_estimate here should be the prior
        //     // unit is ticks / sec
        //     Frequency innovation =
        //       tick_velocity - vel_to_ticks * state_estimate.velocity;
        //     // TODO: update covariance
        // }

        // float covariance

        AngularVelocity innovation =
          geared_motor_reported_velocity - m_state_estimate.velocity;

        CovarianceUnit measurement_covariance =
          m_constants.measurement_covariance_factor *
            units::square(geared_motor_reported_velocity) +
          m_constants.measurement_covariance_offset;

        CovarianceUnit innovation_covariance =
          m_covariance + measurement_covariance;

        float gain = m_covariance / innovation_covariance;

        m_state_estimate.velocity =
          m_state_estimate.velocity + gain * innovation;

        // update covariance from measurement
        m_covariance = (1 - gain) * m_covariance;

        // update timestamp
        // motor_states[idx].last_timestamp = curr_timestamp;
        // motor_states[idx].last_position = raw_position;
    }

    // update input (voltage) based on motor data
    void updateInput() {
        m_input.voltage = 0_volt;
        int motor_count = 0;
        for (auto voltage : motor_group->get_voltage_all()) {
            m_input.voltage += Voltage(voltage) / 12.0;
            motor_count++;
        }
        m_input.voltage /= static_cast<float>(motor_count);
    }

  public:
    // take measurements from the motor(s) and correct model based on them
    void correct() {
        // apply correction for each motor
        for (auto motor : motor_group->get_port_all()) {
            correctSingleMotor(motor);
        }
    }

    // predict x_k+1 from x_k
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

        // V = kv * v + ka * a + ks * sgn(V)
        // [V - sgn(v) * ks] = kv * v + ka * a
        //
        // V_eff = [V - sgn(v) * ks]
        //
        // V_eff = kv * v + ka * a
        //
        // a = (V_eff - kv * v) / ka
        // a = V_eff * 1/ka - (kv/ka) * v

        auto A = -(m_constants.Kv / m_constants.Ka) * dt;
        auto B = (1.0 / m_constants.Ka) * dt;

        // same as accel * dt
        AngularVelocity velocity_delta =
          (B * V_eff + A * m_state_estimate.velocity);

        // assumes constant acceleration:
        // v = v + a * dt
        AngularVelocity new_predicted_v =
          // state_estimate.velocity + state_estimate.acceleration * dt;
          m_state_estimate.velocity + velocity_delta;

        m_state_estimate = { .velocity = new_predicted_v };

        m_covariance = A * A * m_covariance + m_constants.process_covariance;

        // update input after prediction
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
};

class MotorGroupVelocityPlant {
    MotorGroupKalmanFilter filter;
};
