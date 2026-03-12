#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

// TODO: this makes no sense since the torque depends on the gearing of the
// motor.
// pct should be speed / max_speed outputs torque from motor for a
// specific velocity
inline FTorque motor_torque(float pct) {
    // make sure its positive
    pct = std::abs(pct);

    // custom motor model that matches somewhat with recorded drivetrain data
    if (pct >= 1.0) return 0_FNm;

    return 1.85125762613_FNm * (1 - pct);

    // if (pct < 0.532636f) {
    //     return 0.983579_FNm;
    // }
    //
    // if (pct >= 1.13642) return 0_FNm;
    //
    // return -1.62902591131_FNm * pct + 1.85125762613_FNm;
}

inline FTorque motor_torque2(FAngularVelocity x, FAngularVelocity final_rpm) {
    // bunch of dynamics constants
    constexpr Voltage V_max = 11.8_volt;

    // float since its used directly in calculation
    constexpr FTorque T_stall = 0.99144765_Nm;
    constexpr Current I_stall = 2.52_amp;
    constexpr Divided<Torque, Current> Kt = T_stall / I_stall;

    constexpr AngularVelocity w_free_internal = 222.453703704_rpm;
    constexpr Resistance R = 2.5077329288_ohm;
    constexpr Current I_free = 0.1_amp;

    constexpr Divided<AngularVelocity, Voltage> Kv =
      w_free_internal / (V_max - I_free * R);

    // should be 1.85127088733
    constexpr FTorque a_const = (V_max * Kt) / R;
    // should be 0.00814519019666
    constexpr Divided<FTorque, FAngularVelocity> b_const = Kt / (R * Kv);

    // actual calculations
    const float gearing = 200_Frpm / final_rpm;
    const FAngularVelocity adjusted_velocity = x * gearing;

    FTorque torque = a_const - b_const * adjusted_velocity;

    // caps the torque
    if (torque > T_stall) torque = T_stall;

    return torque * gearing;
}

} // namespace lyfast
} // namespace blazing
