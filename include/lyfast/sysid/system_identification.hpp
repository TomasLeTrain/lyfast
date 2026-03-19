#pragma once

#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.h"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <cassert>
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
      std::function<VelUnit()>
        velocity_func, // returns velocity of motor group in velocity units
      std::function<Voltage(Voltage)>
        voltage_func, // returns current voltage, commanded voltage given as
                      // parameter
      std::optional<Time> steady_state_time,
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
    static void printDataAsLatex(const VectorDataT& data);
};

// more explicit instantiations
extern template class MotorGroupUtils<LinearVelocity>;
extern template class MotorGroupUtils<AngularVelocity>;
using LinearMotorGroupUtils = MotorGroupUtils<LinearVelocity>;
using AngularMotorGroupUtils = MotorGroupUtils<AngularVelocity>;

class DifferentialUtils {
  public:
    using VoltageCommandVector = std::vector<DifferentialVoltageCommand>;

    // gathers velocity data from robot by moving voltage_commands
    // does not collect actual voltage data, but rather commanded voltage
    template<typename DrivetrainType>
    static DifferentialData
    generateData(const VoltageCommandVector& voltage_commands,
                 DrivetrainType& drivetrain,
                 Time delta_time,
                 bool use_measured_voltage = false) {
        DifferentialData data;

        uint32_t int_delta_time = std::lround(to_msec(delta_time));

        for (auto [left_voltage, right_voltage, target_time, record] :
             voltage_commands) {
            drivetrain.moveTank(left_voltage, right_voltage);

            auto start_time = blazing::now();
            uint32_t prev_time = pros::millis();

            while (!timeoutDone(target_time, start_time)) {
                // amount of time to measure the steady state
                if (record) {
                    auto [left_velocity, right_velocity] =
                      drivetrain.getDrivetrainVelocities();

                    if (use_measured_voltage) {
                        auto [left_measured_voltage, right_measured_voltage] =
                          drivetrain.getDrivetrainVoltages();

                        data.left.emplace_back(left_velocity,
                                               left_measured_voltage);
                        data.right.emplace_back(right_velocity,
                                                right_measured_voltage);
                    } else {
                        data.left.emplace_back(left_velocity, left_voltage);
                        data.right.emplace_back(right_velocity, right_voltage);
                    }
                }

                pros::c::task_delay_until(&prev_time, int_delta_time);
            }
        }
        return data;
    }

    template<typename DrivetrainType>
    static DifferentialData
    generate_kv_ks_data(const VoltageCommandVector& voltage_commands,
                        DrivetrainType& drivetrain,
                        Time delta_time,
                        Time steady_state_time,
                        bool use_measured_voltage = false) {
        DifferentialData data;

        uint32_t int_delta_time = std::lround(to_msec(delta_time));

        for (auto [left_voltage, right_voltage, target_time, record] :
             voltage_commands) {
            drivetrain.moveTank(left_voltage, right_voltage);

            auto start_time = blazing::now();
            uint32_t prev_time = pros::millis();

            LinearVelocity left_averageVelocities { 0 };
            LinearVelocity right_averageVelocities { 0 };
            Voltage left_averageVoltages { 0 };
            Voltage right_averageVoltages { 0 };
            int samples = 0;

            while (!timeoutDone(target_time, start_time)) {
                // time at which we start to record data
                Time threshold_time =
                  units::max(0_Fsec, target_time - steady_state_time);

                if (timeoutDone(threshold_time, start_time)) {
                    auto [curr_left_vel, curr_right_vel] =
                      drivetrain.getDrivetrainVelocities();

                    left_averageVelocities += curr_left_vel;
                    right_averageVelocities += curr_right_vel;

                    if (use_measured_voltage) {
                        auto [left_measured_voltage, right_measured_voltage] =
                          drivetrain.getDrivetrainVoltages();

                        left_averageVoltages += left_measured_voltage;
                        right_averageVoltages += right_measured_voltage;
                    } else {
                        left_averageVoltages += left_voltage;
                        right_averageVoltages += right_voltage;
                    }

                    samples++;
                }

                pros::c::task_delay_until(&prev_time, int_delta_time);
            }

            if (samples == 0) {
                std::cout << "samples is 0: " << target_time.internal()
                          << std::endl;
            } else {
                left_averageVelocities /= samples;
                left_averageVoltages /= samples;

                right_averageVelocities /= samples;
                right_averageVoltages /= samples;
            }

            if (record) {
                data.left.emplace_back(left_averageVelocities,
                                       left_averageVoltages);
                data.right.emplace_back(right_averageVelocities,
                                        right_averageVoltages);
            }
        }

        return data;
    }

    // return calculated [left,right] kv/ks. Also prints values
    static std::pair<std::pair<KvUnits<LinearVelocity>, KsUnits>,
                     std::pair<KvUnits<LinearVelocity>, KsUnits>>
    calculate_kv_ks(const DifferentialData& data);

    // return calculated [left,right] ka. Also prints values
    static std::pair<KaUnits<LinearVelocity>, KaUnits<LinearVelocity>>
    calculate_ka(const DifferentialData& data,
                 KvUnits<LinearVelocity> left_kv,
                 KsUnits left_ks,
                 KvUnits<LinearVelocity> right_kv,
                 KsUnits right_ks,
                 Time delta_time);

    // return calculated [left,right] ka. Also prints values
    static std::pair<KaUnits<LinearVelocity>, KaUnits<LinearVelocity>>
    calculate_ka_kp_ki_fopdt(const DifferentialData& data,
                             Time delta_time,
                             double lambda_factor);

    static void printData(const DifferentialData& data, Time delta_time);
};
} // namespace sysid
} // namespace lyfast
} // namespace blazing
