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

using FKsUnits = FVoltage;
using FKvUnits = Divided<FVoltage, FLinearVelocity>;
using FKaUnits = Divided<FVoltage, FLinearAcceleration>;

struct OLS_data {
    FLinearVelocity left_velocity;
    FLinearVelocity right_velocity;
    FVoltage left_voltage;
    FVoltage right_voltage;
};

struct SysIdVoltageCommands {
    Voltage left_voltage;
    Voltage right_voltage;
    Time time;
    bool record = true;
};

std::vector<OLS_data>
calculate_kv_ks(std::vector<SysIdVoltageCommands> voltage_commands,
                pros::MotorGroup* left_motors,
                pros::MotorGroup* right_motors,
                Length wheel_diameter,
                AngularVelocity final_rpm);

std::vector<OLS_data>
calculate_ka(std::vector<SysIdVoltageCommands> voltage_commands,
             pros::MotorGroup* left_motors,
             pros::MotorGroup* right_motors,
             Length wheel_diameter,
             AngularVelocity final_rpm,
             KvUnits left_kv,
             KsUnits left_ks,
             KvUnits right_kv,
             KsUnits right_ks);

std::vector<OLS_data>
createData(std::vector<SysIdVoltageCommands> voltage_commands,
           pros::MotorGroup* left_motors,
           pros::MotorGroup* right_motors,
           Length wheel_diameter,
           AngularVelocity final_rpm);

void printData(std::vector<OLS_data> data);

} // namespace lyfast
} // namespace blazing
