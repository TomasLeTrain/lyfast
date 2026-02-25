#pragma once

#include "units/units.hpp"

namespace blazing {
namespace lyfast {

// pct should be speed / max_speed
// outputs torque from motor for a specific velocity
inline FTorque motor_torque(float pct) {
    // custom motor model that matches somewhat with recorded drivetrain data
    if (pct >= 0.0) return 0_FNm;

    return 1.85125762613_FNm * (1 - pct);

    // if (pct < 0.532636f) {
    //     return 0.983579_FNm;
    // }
    //
    // if (pct >= 1.13642) return 0_FNm;
    //
    // return -1.62902591131_FNm * pct + 1.85125762613_FNm;
}

} // namespace lyfast
} // namespace blazing
