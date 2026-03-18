#pragma once

#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <chrono>
#include <map>
#include <mutex>
#include <queue>

namespace blazing {
namespace lyfast {
// kalman filter for a motor group.
// doesnt respond well to disturbances (excess load), ema seems to be good
// enough
class MotorGroupKalmanFilter {
  protected:
    pros::Mutex m_mutex;

  public:
    using CovarianceUnit = Exponentiated<AngularVelocity, std::ratio<2>>;

    struct Input {
        Torque torque;
        Voltage voltage;
    };

    struct State {
        AngularVelocity velocity;
    };

    struct CorrectCovarianceGains {
        // cov = factor * measurement^2
        // covariance increases on larger velocities
        float measurement_covariance_factor = units::square(0.4);
        // minimum covariance
        CovarianceUnit measurement_covariance_offset = units::square(5_rpm);
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

        Divided<AngularVelocity, Torque> Kt;
        // number of ms to disable torque usage
        uint32_t torque_disable_period = 100;

        CovarianceUnit process_covariance = units::square(5_rpm);

        CorrectCovarianceGains motor_reported_gains;
        CorrectCovarianceGains tick_based_gains;
    };

  private:
    pros::MotorGroup* motor_group;

    Constants m_constants;
    State m_state_estimate;
    Input m_input;
    CovarianceUnit m_covariance { 0 };

    uint32_t m_last_predict_timestamp;
    uint32_t m_disabled_torque_timestamp = 0;

    struct motorState {
        uint32_t prev_motor_clock;
        int32_t prev_motor_ticks;
    };

    std::array<std::optional<motorState>, 21> motor_states;

    void correctVelocity(AngularVelocity measurement, bool tick_based) {
        CorrectCovarianceGains& current_constants =
          tick_based ? m_constants.tick_based_gains :
                       m_constants.motor_reported_gains;

        AngularVelocity innovation = measurement - m_state_estimate.velocity;

        CovarianceUnit measurement_covariance =
          current_constants.measurement_covariance_factor *
            units::square(measurement) +
          current_constants.measurement_covariance_offset;

        CovarianceUnit innovation_covariance =
          m_covariance + measurement_covariance;

        float gain = m_covariance / innovation_covariance;

        m_state_estimate.velocity =
          m_state_estimate.velocity + gain * innovation;

        // update covariance from measurement
        m_covariance = (1 - gain) * m_covariance;
    }

    void correctSingleMotor(std::uint8_t idx) {
        // AngularVelocity raw_motor_measurement =
        //   motor_group->get_actual_velocity(idx) * rpm;
        // pros::MotorGears gearing = motor_group->get_gearing(idx);

        // AngularVelocity motor_measurement =
        //   (raw_motor_measurement / gearingToVelocity(gearing)) *
        //   m_constants.final_gearing_rpm;

        // correct using the motor estimated velocity
        // TODO: disabled for debugging for now
        // correctVelocity(motor_measurement, false);

        uint32_t curr_motor_clock;
        int32_t curr_motor_ticks =
          motor_group->get_raw_position(&curr_motor_clock, idx);

        uint32_t motor_port = std::abs(motor_group->get_port(idx)) - 1;
        auto& motor_state = motor_states[motor_port];

        if (!motor_state.has_value()) {
            // no clue on the previous value, update previous and skip
            motor_state = motorState { .prev_motor_clock = curr_motor_clock,
                                       .prev_motor_ticks = curr_motor_ticks };
            return;
        }

        double motor_dt =
          5.0 *
          std::round((curr_motor_clock - motor_state->prev_motor_clock) / 5.0);

        double tick_delta = curr_motor_ticks - motor_state->prev_motor_ticks;
        motor_state->prev_motor_clock = curr_motor_clock;
        motor_state->prev_motor_ticks = curr_motor_ticks;

        // 900 ticks / revolution for 200 rpm cart
        // ticks decrease as final rpm increases
        Divided<Number, Angle> conversion =
          (900.0 / rot) * (200_rpm / m_constants.final_gearing_rpm);

        Angle angular_position_delta = tick_delta / conversion;
        AngularVelocity tick_based_measurement =
          angular_position_delta / from_msec(motor_dt);

        if (std::abs(motor_dt) <= 1e-5 ||
            units::abs(tick_based_measurement) >
              29_radps) { // Motor position reset manually
            // again don't have any new information, return
        } else {
            // we do have good measurement, use
            correctVelocity(tick_based_measurement, true);
        }
    }

    void updateTorqueInput() {
        int motor_count = 0;

        m_input.torque = 0_Nm;
        for (auto torque : motor_group->get_torque_all()) {
            m_input.torque += from_Nm(torque);
            motor_count++;
        }

        m_input.torque /= static_cast<float>(motor_count);
    }

    // update input (voltage) based on motor data
    void updateInput() {
        int motor_count = 0;

        m_input.voltage = 0_volt;
        for (auto voltage : motor_group->get_voltage_all()) {
            m_input.voltage += from_mvolt(voltage) / 12.0;
            motor_count++;
        }
        m_input.voltage /= static_cast<float>(motor_count);

        updateTorqueInput();
    }

  public:
    // take measurements from the motor(s) and correct model based on them
    void correct() {
        std::lock_guard lock(m_mutex);
        // apply correction for each motor
        for (int i = 0; i < motor_group->size(); i++) {
            correctSingleMotor(i);
        }
    }

    // predict x_k+1 from x_k and u_k
    void predict(Time dt) {
        std::lock_guard lock(m_mutex);
        // torque data seems to be delayed, so use current one instead of last
        // data
        updateTorqueInput();

        Voltage V_eff;
        if (units::abs(m_input.voltage) < m_constants.Ks) {
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
        // v = v + a * t
        // v = v + (V_eff * 1/ka - (kv/ka) * v) * dt
        // v = v(1 - (kv/ka) * dt) + V_eff * (1/ka * dt)
        //
        // A = 1 - (kv/ka) * dt
        // B = 1/ka * dt
        //
        // v = A * v + B * V_eff

        auto curr_time = pros::millis();
        bool curr_torque_enabled = curr_time - m_disabled_torque_timestamp >
                                   m_constants.torque_disable_period;

        double A = 1 - (m_constants.Kv / m_constants.Ka) * dt;
        auto Bv = (1.0 / m_constants.Ka) * dt;
        auto Bt = -m_constants.Kt * units::sgn(V_eff);

        AngularVelocity new_predicted_v =
          m_state_estimate.velocity * A + Bv * V_eff;

        if (curr_torque_enabled) new_predicted_v += Bt * m_input.torque;

        m_state_estimate = { .velocity = new_predicted_v };

        m_covariance = A * A * m_covariance + m_constants.process_covariance;

        // save voltage before updating input
        auto last_voltage = m_input.voltage;

        // update input after prediction
        updateInput();

        auto voltage_over_time = (m_input.voltage - last_voltage) / dt;

        // TODO: move 3.5 to be a constant
        if (units::abs(voltage_over_time) > Divided<Voltage, Time> { 3.5 }) {
            // there will be a torque spike because of the voltage change,
            // disable torque based prediction temporarily to avoid wrong
            // predictions
            m_disabled_torque_timestamp = curr_time;
            std::cout << "spike detected" << std::endl;
        }

        m_last_predict_timestamp = curr_time;
    }

    // predicts to match the timestamp, as a time in pros::millis()
    void predictToTimestamp(uint32_t timestamp) {
        if (timestamp <= m_last_predict_timestamp) {
            // timestamp before the latest prediction timestamp, can't predict
            // into the past
            return;
        }
        predict(from_msec(timestamp - m_last_predict_timestamp));
    }

    uint32_t getLastPredictTimestamp() {
        std::lock_guard lock(m_mutex);
        return m_last_predict_timestamp;
    }

    Input getInput() {
        std::lock_guard lock(m_mutex);
        return m_input;
    }

    State getPredictedState() {
        std::lock_guard lock(m_mutex);
        return m_state_estimate;
    }

    void setPredictedState(State new_state, CovarianceUnit covariance) {
        std::lock_guard lock(m_mutex);
        m_state_estimate = new_state;
        m_covariance = covariance;
        m_last_predict_timestamp = pros::millis();
    }

    MotorGroupKalmanFilter(pros::MotorGroup* motors,
                           Constants constants,
                           State initial_state_estimate,
                           CovarianceUnit initial_covariance)
        : motor_group(motors),
          m_constants(constants),
          m_state_estimate(initial_state_estimate),
          m_input({ .torque = 0_Nm, .voltage = 0_volt }),
          m_covariance(initial_covariance),
          m_last_predict_timestamp(pros::millis()) {}
};

} // namespace lyfast
} // namespace blazing
