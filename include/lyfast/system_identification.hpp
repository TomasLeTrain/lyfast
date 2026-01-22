#pragma once

#include "blazing/utils.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <numeric>
#include <print>
#include <ranges>
#include <utility>

namespace blazing {
namespace lyfast {

// voltage is assumed to be in the range [0,1]

// u = Ks * sgn(v) + Kv * v + Ka * a;
using KsUnits = Voltage;
using KvUnits = Divided<Voltage, LinearVelocity>;
using KaUnits = Divided<Voltage, LinearAcceleration>;

struct OLS_data {
    LinearVelocity left_velocity;
    LinearVelocity right_velocity;
    Voltage left_voltage;
    Voltage right_voltage;
};

struct SysIdVoltageCommands {
    Voltage left_voltage;
    Voltage right_voltage;
    Time time;
    bool record = true;
};

void calculate_kv_ks();

std::vector<OLS_data>
createData(std::vector<SysIdVoltageCommands> voltage_commands,
           pros::MotorGroup* left_motors,
           pros::MotorGroup* right_motors,
           Length wheel_diameter,
           AngularVelocity final_rpm);

void printData(std::vector<OLS_data> data);

} // namespace lyfast
} // namespace blazing
