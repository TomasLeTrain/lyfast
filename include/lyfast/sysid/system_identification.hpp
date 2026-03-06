#pragma once

#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <functional>
#include <numeric>
#include <print>
#include <ranges>
#include <utility>

namespace blazing {
namespace lyfast {
namespace sysid {

template<typename T>
// holds a velocity and an associated voltage
struct SysidEntry {
    ConvertFloatType<T, float> velocity;
    FVoltage voltage;
};

// explicit instantiations
extern template struct SysidEntry<LinearVelocity>;
extern template struct SysidEntry<AngularVelocity>;

using LinearSysidEntry = SysidEntry<LinearVelocity>;
using AngularSysidEntry = SysidEntry<AngularVelocity>;

// struct of arrays holding data for left and right
struct DifferentialData {
    std::vector<LinearSysidEntry> left;
    std::vector<LinearSysidEntry> right;
};

struct VoltageCommand {
    FVoltage voltage;

    FTime time;
    bool record = true;
};

struct DifferentialVoltageCommand {
    FVoltage left_voltage;
    FVoltage right_voltage;

    FTime time;
    bool record = true;
};

template<typename VelUnit>
class MotorGroupUtils {
  public:
    using AccelUnit = Divided<VelUnit, Time>;
    using DataT = SysidEntry<VelUnit>;
    using VectorDataT = std::vector<DataT>;
    using ka_ki_kp_dataT = std::tuple<
      // T
      Time,
      // K
      Divided<VelUnit, Voltage>,
      // Ka
      KaUnits<VelUnit>,
      // Kp
      KpUnits<VelUnit>,
      // Ki
      KiUnits<VelUnit>>;

    // gathers velocity data from robot by moving voltage_commands
    static VectorDataT generateData(
      std::vector<VoltageCommand> voltage_commands,
      pros::MotorGroup* motor_group,
      AngularVelocity final_rpm,
      std::function<VelUnit(AngularVelocity)>
        conversion_func, // converts drivetrain rpm to vel unit
      std::optional<Time>
        steady_state_time, // if specified it only captures data at the end of
                           // the voltage command for steady_state_time time
      Time delta_time);

    // uses linear least squares to find kv and ks gains that best fit data
    // for best results only use steady state velocity data
    static std::pair<KvUnits<VelUnit>, KsUnits>
    fit_kv_ks_data(const VectorDataT& data);

    // calculates ka/kp/ki from T and K constants and lambda factor
    static auto calculate_ka_kp_ki_from_TK(Time dt,
                                           Divided<VelUnit, Voltage> K,
                                           double lambda_factor)
      -> std::tuple<
        // Ka
        KaUnits<VelUnit>,
        // Kp
        KpUnits<VelUnit>,
        // Ki
        KiUnits<VelUnit>>;

    // fits various values data using model:
    // accel = voltage * h - velocity * g
    static ka_ki_kp_dataT fit_ka_kp_ki_data_first_model(const VectorDataT& data,
                                                        Time delta_time,
                                                        double lambda_factor);

    // fits data using different model:
    // velocity_next = a1 * velocity + a2 * voltage
    //
    static ka_ki_kp_dataT
    fit_ka_kp_ki_data_second_model(const VectorDataT& data,
                                   Time delta_time,
                                   double lambda_factor);

    // averages results from both models. This seems to produce really good
    // results as both deviate in opposite directions
    static ka_ki_kp_dataT fit_ka_kp_ki_data_both_models(const VectorDataT& data,
                                                        Time delta_time,
                                                        double lambda_factor);

    // fits data specifically only for ka with model:
    // accel * ka = voltage - velocity * kv - sgn(velocity) * ks
    static KaUnits<VelUnit> fit_ka_data(const VectorDataT& data,
                                        Time delta_time,
                                        KvUnits<VelUnit> kv,
                                        KsUnits ks);

    // print data in desmos-friendly format - prints in VelUnit units
    static void print_data_as_latex(const VectorDataT& data);
};

// more explicit instantiations
extern template class MotorGroupUtils<LinearVelocity>;
extern template class MotorGroupUtils<AngularVelocity>;
using LinearMotorGroupUtils = MotorGroupUtils<LinearVelocity>;
using AngularMotorGroupUtils = MotorGroupUtils<AngularVelocity>;

class DifferentialUtils {
  public:
    // gathers velocity data from robot by moving voltage_commands
    // does not collect actual voltage data, but rather commanded voltage
    static DifferentialData
    createData(std::vector<DifferentialVoltageCommand> voltage_commands,
               DifferentialDrivetrain& drivetrain,
               Time delta_time);

    static DifferentialData
    gather_kv_ks_data(std::vector<DifferentialVoltageCommand> voltage_commands,
                      DifferentialDrivetrain& drivetrain,
                      Time delta_time,
                      Time steady_state_time);

    static DifferentialData
    calculate_kv_ks(std::vector<DifferentialVoltageCommand> voltage_commands,
                    DifferentialDrivetrain& drivetrain,
                    Time delta_time = 10_msec,
                    Time steady_state_time = 200_msec);

    static DifferentialData
    calculate_ka(std::vector<DifferentialVoltageCommand> voltage_commands,
                 DifferentialDrivetrain& drivetrain,
                 KvUnits<LinearVelocity> left_kv,
                 KsUnits left_ks,
                 KvUnits<LinearVelocity> right_kv,
                 KsUnits right_ks,
                 Time delta_time);

    static void printData(DifferentialData data, Time delta_time);

    static DifferentialData
    calculate_ka_kp_ki_fopdt(DifferentialVoltageCommand voltage_command,
                             DifferentialDrivetrain& drivetrain,
                             Time delta_time,
                             double lambda_factor);
};
} // namespace sysid
} // namespace lyfast
} // namespace blazing
