#pragma once

#include "pros/apix.h"
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

template<typename floatType>
struct LeftRightSpeedsT {
    ConvertFloatType<LinearVelocity, floatType> left_vel;
    ConvertFloatType<LinearVelocity, floatType> right_vel;

    constexpr LeftRightSpeedsT& operator+=(const LeftRightSpeedsT& rhs) {
        left_vel += rhs.left_vel;
        right_vel += rhs.right_vel;
        return *this;
    }

    constexpr LeftRightSpeedsT& operator-=(const LeftRightSpeedsT& rhs) {
        left_vel -= rhs.left_vel;
        right_vel -= rhs.right_vel;
        return *this;
    }
};

template<typename floatType>
struct LeftRightVoltagesT {
    ConvertFloatType<Voltage, floatType> left_voltage;
    ConvertFloatType<Voltage, floatType> right_voltage;

    constexpr LeftRightVoltagesT& operator+=(const LeftRightVoltagesT& rhs) {
        left_voltage += rhs.left_voltage;
        right_voltage += rhs.right_voltage;
        return *this;
    }

    constexpr LeftRightVoltagesT& operator-=(const LeftRightVoltagesT& rhs) {
        left_voltage -= rhs.left_voltage;
        right_voltage -= rhs.right_voltage;
        return *this;
    }
};

template<typename floatType>
struct DifferentialSpeedsT {
    ConvertFloatType<LinearVelocity, floatType> linear_velocity;
    ConvertFloatType<AngularVelocity, floatType> angular_velocity;

    constexpr DifferentialSpeedsT& operator+=(const DifferentialSpeedsT& rhs) {
        linear_velocity += rhs.linear_velocity;
        angular_velocity += rhs.angular_velocity;
        return *this;
    }

    constexpr DifferentialSpeedsT& operator-=(const DifferentialSpeedsT& rhs) {
        linear_velocity -= rhs.linear_velocity;
        angular_velocity -= rhs.angular_velocity;
        return *this;
    }
};

template<typename floatType>
struct DifferentialVoltagesT {
    ConvertFloatType<Voltage, floatType> linear_voltage;
    ConvertFloatType<Voltage, floatType> angular_voltage;

    constexpr DifferentialVoltagesT&
    operator+=(const DifferentialVoltagesT& rhs) {
        linear_voltage += rhs.linear_voltage;
        angular_voltage += rhs.angular_voltage;
        return *this;
    }

    constexpr DifferentialVoltagesT&
    operator-=(const DifferentialVoltagesT& rhs) {
        linear_voltage -= rhs.linear_voltage;
        angular_voltage -= rhs.angular_voltage;
        return *this;
    }
};

#define SpeedOps(T)                                             \
    template<typename floatType>                                \
    constexpr T<floatType> operator+(T<floatType> lhs,          \
                                     const T<floatType>& rhs) { \
        return lhs += rhs;                                      \
    }                                                           \
    template<typename floatType>                                \
    constexpr T<floatType> operator-(T<floatType> lhs,          \
                                     const T<floatType>& rhs) { \
        return lhs -= rhs;                                      \
    }

SpeedOps(LeftRightSpeedsT);
SpeedOps(LeftRightVoltagesT);
SpeedOps(DifferentialSpeedsT);
SpeedOps(DifferentialVoltagesT);

// TODO: could possibly explicitly instantiate?
using LeftRightSpeeds = LeftRightSpeedsT<double>;
using FLeftRightSpeeds = LeftRightSpeedsT<float>;

using LeftRightVoltages = LeftRightVoltagesT<double>;
using FLeftRightVoltages = LeftRightVoltagesT<float>;

using DifferentialSpeeds = DifferentialSpeedsT<double>;
using FDifferentialSpeeds = DifferentialSpeedsT<float>;

using DifferentialVoltages = DifferentialVoltagesT<double>;
using FDifferentialVoltages = DifferentialVoltagesT<float>;

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

    T largest_magnitude =
      units::abs(*std::ranges::max_element(saturated, abs_compare));
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

// gets the average angular velocity of the motor group
inline AngularVelocity get_group_velocity(pros::MotorGroup* motors,
                                          AngularVelocity final_rpm) {
    AngularVelocity average_rpm = 0_rpm;

    for (std::int8_t motor_i = 0; motor_i < motors->size(); motor_i++) {
        auto zero_indexed_port = abs(motors->get_port(motor_i)) - 1;
        bool installed = pros::DeviceType::motor ==
                         (pros::DeviceType)pros::c::registry_get_plugged_type(
                           zero_indexed_port);
        if (!installed) continue;

        double velocity = motors->get_actual_velocity(motor_i);
        pros::MotorGears encoder_units = motors->get_gearing(motor_i);
        AngularVelocity start_rpm = gearingToVelocity(encoder_units);
        AngularVelocity actual_rpm = (velocity * rpm) * (final_rpm / start_rpm);

        average_rpm += actual_rpm;
    }

    average_rpm /= motors->size();

    return average_rpm;
};

// gets the average linear velocity of the motor group
LinearVelocity get_group_velocity(pros::MotorGroup* motors,
                                  Length wheel_diameter,
                                  AngularVelocity final_rpm);

// gets the average voltage of the motor group
Voltage get_group_voltage(pros::MotorGroup* motors);

} // namespace blazing
