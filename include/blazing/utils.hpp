#pragma once

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <array>
#include <optional>

namespace blazing {
enum class AngularDirection {
    // equal to CCW
    LEFT,
    // equal to CW
    RIGHT
};

struct LeftRightVoltages {
    Voltage left_voltage;
    Voltage right_voltage;
};

struct LeftRightSpeeds {
    LinearVelocity left_vel;
    LinearVelocity right_vel;
};

struct DifferentialSpeeds {
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
};

struct DifferentialVoltages {
    Voltage linear_voltage;
    Voltage angular_voltage;
};

// returns time since program started
// uses pros::millis to get the information
Time now();

Divided<Number, Angle> sinc(Angle theta);

/**
 * @brief Determines the smallest signed error between two arbitrary angles. In
 * other words, heading + angleError(target,heading) = target.  If direction is
 * set, it returns the smallest error that can be achieved while only travelling
 * in that direction
 *
 * @param target target heading
 * @param heading heading
 * @param direction
 * @return angle error
 */
Angle angleError(Angle target,
                 Angle heading,
                 std::optional<AngularDirection> direction = std::nullopt);

// returns opposite angle, in the range [0,2pi)
Angle reverseAngle(Angle angle);

// calculates delta time given some last time (which can be nullopt)
// also updates last_time to equal current time
// NOTE: returns 0 if last_time is nullopt!
Time deltaTime(std::optional<Time>& last_time);

// determines if a timeout has triggered given a start time
bool timeoutDone(std::optional<Time> timeout, Time start_time);

// same as units::sgn, but returns 1.0 if the number is equal to zero (never
// returns 0 for the sign)
template<isQuantity Q>
Number signed_sgn(Q num) {
    return num.internal() >= 0.0 ? Number(1.0) : Number(-1.0);
}

// scales all values of saturated such that max(desaturated) <= max
template<isQuantity T, size_t size>
std::array<T, size> desaturate(std::array<T, size> saturated, T max) {

    auto abs_compare = [](T a, T b) {
        return units::abs(a) < units::abs(b);
    };

    T largest_magnitude = *std::ranges::max_element(saturated, abs_compare);
    Number multiplier = max / largest_magnitude;

    if (largest_magnitude > max) {
        std::transform(saturated.cbegin(),
                       saturated.cend(),
                       saturated.begin(),
                       [multiplier](T num) {
                           return num * multiplier;
                       });
    };

    return saturated;
}

inline LinearVelocity get_group_velocity(pros::MotorGroup* motor_group,
                                         Length wheel_diameter,
                                         AngularVelocity final_rpm) {
    AngularVelocity average_rpm = 0_rpm;

    for (std::int8_t motor_i = 0; motor_i < motor_group->size(); motor_i++) {
        double velocity = motor_group->get_actual_velocity(motor_i);
        pros::MotorGears encoder_units = motor_group->get_gearing(motor_i);
        AngularVelocity start_rpm;

        switch (encoder_units) {
            case pros::MotorGears::blue: start_rpm = 600_rpm; break;
            case pros::MotorGears::green: start_rpm = 200_rpm; break;
            case pros::MotorGears::red: start_rpm = 100_rpm; break;
            default: 200_rpm; break;
        }

        AngularVelocity actual_rpm = (velocity * rpm) * final_rpm / start_rpm;

        average_rpm += actual_rpm;
    }

    average_rpm /= motor_group->size();

    LinearVelocity velocity = average_rpm * (wheel_diameter * M_PI) / rot;

    return velocity;
};

inline Voltage get_group_voltage(pros::MotorGroup* motors) {
    Voltage result = 0_volt;
    for (auto voltage : motors->get_voltage_all()) {
        result += from_mvolt(voltage) / 12;
    }
    result /= motors->size();
    return result;
};

} // namespace blazing
