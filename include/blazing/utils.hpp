#pragma once

#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <array>
#include <optional>

namespace blazing {
enum class AngularDirection {
    LEFT,
    RIGHT
};


struct DifferentialVoltages {
    Voltage left_voltage;
    Voltage right_voltage;
};

struct DifferentialSpeeds {
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
};

// returns time since program started
// uses pros::millis to get the information
Time now();

Divided<Number, Angle> sinc(Angle theta);

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

} // namespace blazing
