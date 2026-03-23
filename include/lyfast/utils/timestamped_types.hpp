#pragma once

#include "units/units.hpp"
#include <cstdint>

template<typename T>
struct TimestampedVelocity {
    T velocity;
    uint32_t timestamp;

    bool operator<(const TimestampedVelocity& rhs) {
        return timestamp < rhs.timestamp;
    }
};
