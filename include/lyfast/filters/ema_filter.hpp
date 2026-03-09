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
#include <queue>

namespace blazing {
namespace lyfast {

// returns -1 if gearing is invalid

class EMAVelocityFilter {
  public:
    using Input = Voltage;
    using State = AngularVelocity;

    struct Constants {
        // the supposed final rpm after applying gearing. For example:
        // 600 rpm goes through 2:1 gear ratio, now becomes 1200 rpm (final
        // gearing rpm)
        AngularVelocity final_gearing_rpm;

        // minimum alpha
        float Koffset = 0.1;
        float KalphaFactor = 0.7;
    };

  private:
    pros::MotorGroup* motor_group;

    Constants m_constants;
    State m_state_estimate;
    Input m_input;

    uint32_t m_last_predict_timestamp;

    // calculated during predict
    float m_alpha_gain = 1.0;

    struct motorState {
        uint32_t prev_motor_clock;
        int32_t prev_motor_ticks;
    };

    std::array<std::optional<motorState>, 21> motor_states;

    void correctVelocity(AngularVelocity measurement) {
        AngularVelocity innovation = measurement - m_state_estimate;

        m_state_estimate = m_state_estimate + m_alpha_gain * innovation;
    }

    void correctSingleMotor(std::uint8_t idx) {
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

        auto gearing_factor = (200_rpm / m_constants.final_gearing_rpm);

        // 900 ticks / revolution for 200 rpm cart
        // ticks decrease as final rpm increases
        Divided<Number, Angle> conversion = (900.0 / rot) * gearing_factor;

        Angle angular_position_delta = tick_delta / conversion;
        AngularVelocity tick_based_measurement =
          angular_position_delta / from_msec(motor_dt);

        if (std::abs(motor_dt) <= 1e-5 ||
            units::abs(tick_based_measurement) >
              // if measurement is impossibly fast
              m_constants.final_gearing_rpm * 2.0) {
            // Motor position was likely reset, reset manually or dc'd
            // again don't have any new information, return
        } else {
            // we do have good measurement, use
            correctVelocity(tick_based_measurement);
        }
    }

    // update input (voltage) based on motor data
    void updateInput() {
        int motor_count = 0;

        m_input = 0_volt;
        for (auto voltage : motor_group->get_voltage_all()) {
            m_input += from_mvolt(voltage) / 12.0;
            motor_count++;
        }
        m_input /= static_cast<float>(motor_count);
    }

  public:
    // take measurements from the motor(s) and correct model based on them
    void correct() {
        // apply correction for each motor
        for (int i = 0; i < motor_group->size(); i++) {
            correctSingleMotor(i);
        }
    }

    // uses input to calculate gain for the current iteration
    // (constant model)
    void predict(Time dt) {
        // can't go back in time or predict to where we are right now
        if (dt.convert(msec) <= 1e-6) return;

        Voltage last_V = m_input;
        updateInput();
        Voltage curr_V = m_input;

        float voltage_derivative = ((curr_V - last_V) / dt).internal();

        m_alpha_gain =
          m_constants.Koffset +
          // uses sqrt as scaling function for the derivative, could test others
          // like ln
          sqrt(units::abs(voltage_derivative)) * m_constants.KalphaFactor;

        m_alpha_gain = units::clamp(m_alpha_gain, 0, 1);

        m_last_predict_timestamp = pros::millis();
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
        return m_last_predict_timestamp;
    }

    Input getInput() {
        return m_input;
    }

    State getPredictedState() {
        return m_state_estimate;
    }

    void setPredictedState(State new_state) {
        m_state_estimate = new_state;
        m_last_predict_timestamp = pros::millis();
    }

    EMAVelocityFilter(pros::MotorGroup* motors,
                      Constants constants,
                      State initial_state_estimate = 0_radps)
        : motor_group(motors),
          m_constants(constants),
          m_state_estimate(initial_state_estimate),
          m_input(0_volt),
          m_last_predict_timestamp(pros::millis()) {}
};

} // namespace lyfast
} // namespace blazing
