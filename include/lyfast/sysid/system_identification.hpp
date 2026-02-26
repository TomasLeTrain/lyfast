#pragma once

#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
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

struct MotorSysidData {
    FLinearVelocity velocity;
    FVoltage voltage;
};

struct MotorSysidVoltageCommands {
    FVoltage voltage;

    FTime time;
    bool record = true;
};

struct DifferentialSysidData {
    std::vector<MotorSysidData> left;
    std::vector<MotorSysidData> right;
};

struct OLS_data {
    FLinearVelocity left_velocity;
    FLinearVelocity right_velocity;
    FVoltage left_voltage;
    FVoltage right_voltage;
};

struct DifferentialSysIdVoltageCommands {
    FVoltage left_voltage;
    FVoltage right_voltage;

    FTime time;
    bool record = true;
};

class MotorGroupSysid {
  public:
    // gathers velocity data from robot by moving voltage_commands
    static std::vector<MotorSysidData>
    generateData(std::vector<MotorSysidVoltageCommands> voltage_commands,
                 pros::MotorGroup* motor_group,
                 Length wheel_diameter,
                 AngularVelocity final_rpm,
                 Time delta_time);

    // uses linear least squares to find kv and ks gains that best fit data
    // for best results only use steady state velocity data
    static std::pair<KvUnits, KsUnits>
    fit_kv_ks_data(std::vector<MotorSysidData> data);

    // calculates ka/kp/ki from T and K constants and lambda factor
    static auto calculate_ka_kp_ki_from_TK(Time T,
                                           Divided<LinearVelocity, Voltage> K,
                                           double lambda_factor)
      -> std::tuple<
        // Ka
        KaUnits,
        // Kp
        Divided<Voltage, LinearVelocity>,
        // Ki
        Divided<Voltage, Length>>;

    // fits various values data using model:
    // accel = voltage * h - velocity * g
    static auto fit_ka_kp_ki_data_first_model(std::vector<MotorSysidData> data,
                                              Time delta_time,
                                              double lambda_factor)
      -> std::tuple<
        // T
        Time,
        // K
        Divided<LinearVelocity, Voltage>,
        // Ka
        KaUnits,
        // Kp
        Divided<Voltage, LinearVelocity>,
        // Ki
        Divided<Voltage, Length>>;

    // fits data using different model:
    // velocity_next = a1 * velocity + a2 * voltage
    //
    static auto fit_ka_kp_ki_data_second_model(std::vector<MotorSysidData> data,
                                               Time delta_time,
                                               double lambda_factor)
      -> std::tuple<
        // T
        Time,
        // K
        Divided<LinearVelocity, Voltage>,
        // Ka
        KaUnits,
        // Kp
        Divided<Voltage, LinearVelocity>,
        // Ki
        Divided<Voltage, Length>>;

    // averages results from both models. This seems to produce really good
    // results as both deviate in opposite directions
    static auto fit_ka_kp_ki_data_both_models(std::vector<MotorSysidData> data,
                                              Time delta_time,
                                              double lambda_factor)
      -> std::tuple<
        // T
        Time,
        // K
        Divided<LinearVelocity, Voltage>,
        // Ka
        KaUnits,
        // Kp
        Divided<Voltage, LinearVelocity>,
        // Ki
        Divided<Voltage, Length>>;

    static KaUnits fit_ka_data(std::vector<MotorSysidData> data,
                               Time delta_time,
                               KvUnits kv,
                               KsUnits ks);
    // print data in desmos-friendly format
    static void print_data_as_latex(std::vector<MotorSysidData>& data);
};

class DifferentialSysid {
  public:
    // gathers velocity data from robot by moving voltage_commands
    // does not collect actual voltage data, but rather commanded voltage
    static DifferentialSysidData
    createData(std::vector<DifferentialSysIdVoltageCommands> voltage_commands,
               DifferentialDrivetrain& drivetrain,
               Time delta_time);

    static DifferentialSysidData gather_kv_ks_data(
      std::vector<DifferentialSysIdVoltageCommands> voltage_commands,
      DifferentialDrivetrain& drivetrain,
      Time delta_time,
      Time steady_state_time);

    static DifferentialSysidData calculate_kv_ks(
      std::vector<DifferentialSysIdVoltageCommands> voltage_commands,
      DifferentialDrivetrain& drivetrain,
      Time delta_time = 10_msec,
      Time steady_state_time = 200_msec);

    static DifferentialSysidData
    calculate_ka(std::vector<DifferentialSysIdVoltageCommands> voltage_commands,
                 DifferentialDrivetrain& drivetrain,
                 KvUnits left_kv,
                 KsUnits left_ks,
                 KvUnits right_kv,
                 KsUnits right_ks,
                 Time delta_time);

    static void printData(DifferentialSysidData data, Time delta_time);

    // NOTE: uses first model right now, can change later if other model is
    // verified to work
    static DifferentialSysidData
    calculate_ka_kp_ki_fopdt(DifferentialSysIdVoltageCommands voltage_command,
                             DifferentialDrivetrain& drivetrain,
                             Time delta_time,
                             double lambda_factor);
};
} // namespace lyfast
} // namespace blazing
