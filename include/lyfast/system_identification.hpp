#pragma once

#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

// voltage is assumed to be in the range [0,1]

// u = Ks * sgn(v) + Kv * v + Ka * a;
using KsUnits = Voltage;
using KvUnits = Divided<Voltage, LinearVelocity>;
using KaUnits = Divided<Voltage, LinearAcceleration>;



} // namespace lyfast
} // namespace blazing
