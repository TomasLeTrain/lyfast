#pragma once

#include "units/units.hpp"

namespace blazing {
namespace lyfast {

// pct should be speed / max_speed
// outputs torque from motor for a specific velocity
inline FTorque motor_torque(float pct) {
    if (pct < 0.532636f) {
        return 0.983579_FNm;
    }

    return -1.62902591131_FNm * pct + 1.85125762613_FNm;
}

} // namespace lyfast
} // namespace blazing
