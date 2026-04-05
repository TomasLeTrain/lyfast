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
FTime Fnow();

Time nowMicro();
FTime FnowMicro();

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
units::conditionalNumber<typename Q::floatType> signed_sgn(Q num) {
    using NumberT = units::conditionalNumber<typename Q::floatType>;
    return num.internal() >= 0.0 ? NumberT(1.0) : NumberT(-1.0);
}

// scales all values of saturated such that max(desaturated) <= max
template<isQuantity T, size_t size>
std::array<T, size> desaturate(std::array<T, size> saturated, T max) {
    auto abs_compare = [](const T& a, const T& b) {
        return units::abs(a) < units::abs(b);
    };

    T largest_magnitude =
      units::abs(*std::ranges::max_element(saturated, abs_compare));
    Number multiplier = max / largest_magnitude;

    if (largest_magnitude > max) {
        std::transform(saturated.cbegin(),
                       saturated.cend(),
                       saturated.begin(),
                       [multiplier](const T& num) -> T {
                           return num * multiplier;
                       });
    };
    return saturated;
}

// returns gearing of the motor as angular velocity
FAngularVelocity gearingToVelocity(pros::MotorGears gearing);

// gets the average angular velocity of the motor group
FAngularVelocity getGroupVelocity(pros::MotorGroup* motors,
                                  FAngularVelocity final_rpm);

// gets the average linear velocity of the motor group
FLinearVelocity getGroupVelocity(pros::MotorGroup* motors,
                                 FLength wheel_diameter,
                                 FAngularVelocity final_rpm);

// gets the average voltage of the motor group
FVoltage getGroupVoltage(pros::MotorGroup* motors);

} // namespace blazing
