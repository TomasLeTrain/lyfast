#include "lyfast/sysid/system_identification.hpp"
#include "Eigen/Dense"
#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <vector>

namespace blazing {
namespace lyfast {

// gathers velocity data from robot by moving voltage_commands
template<typename T>
MotorGroupSysid<T>::VectorDataT MotorGroupSysid<T>::generateData(
  std::vector<MotorSysidVoltageCommands> voltage_commands,
  pros::MotorGroup* motor_group,
  AngularVelocity final_rpm,
  std::function<T(AngularVelocity)> conversionFunc,
  Time delta_time) {

    uint32_t int_delta_time = std::lround(to_msec(delta_time));
    VectorDataT data;

    for (auto [voltage, target_time, record] : voltage_commands) {
        motor_group->move_voltage(12 * to_mvolt(voltage));

        auto start_time = blazing::now();
        uint32_t prev_time = pros::millis();

        while (!timeoutDone(target_time, start_time)) {
            // amount of time to measure the steady state
            if (record) {
                T velocity = conversionFunc(
                  blazing::get_group_velocity(motor_group, final_rpm));

                data.emplace_back(velocity, voltage);
            }

            pros::c::task_delay_until(&prev_time, int_delta_time);
        }
    }
    return data;
}

// uses linear least squares to find kv and ks gains that best fit data
// for best results only use steady state velocity data
template<typename T>
std::pair<KvUnits<T>, KsUnits>
MotorGroupSysid<T>::fit_kv_ks_data(const VectorDataT& data) {
    // of form < velocity, sgn(velocity) >
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(data.size(), 2);
    // of form < Voltage >
    Eigen::VectorXd b = Eigen::VectorXd::Zero(data.size());

    for (size_t i = 0; i < data.size(); i++) {
        DataT& curr = data[i];

        A(i, 0) = curr.velocity.internal();
        A(i, 1) = units::sgn(curr.velocity);

        b(i) = curr.voltage.internal();
    }

    Eigen::VectorXd solution =
      A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    KvUnits<T> kv { solution(0) };
    KsUnits ks { solution(1) };
    return { kv, ks };
}

// calculates ka/kp/ki from T and K constants and lambda factor
template<typename T>
auto MotorGroupSysid<T>::calculate_ka_kp_ki_from_TK(
  Time _T,
  Divided<LinearVelocity, Voltage> K,
  double lambda_factor)
  -> std::tuple<
    // Ka
    KaUnits<T>,
    // Kp
    Divided<Voltage, LinearVelocity>,
    // Ki
    Divided<Voltage, Length>> {

    KaUnits<T> Ka = _T / K;

    Time lambda = lambda_factor * _T;

    Divided<Voltage, LinearVelocity> Kp = _T / (K * lambda);
    Divided<Voltage, Length> Ki = Kp / _T;

    return { Ka, Kp, Ki };
}

// fits various values data using model:
// accel = voltage * h - velocity * g
template<typename T>
auto MotorGroupSysid<T>::fit_ka_kp_ki_data_first_model(const VectorDataT& data,
                                                       Time delta_time,
                                                       double lambda_factor)
  -> std::tuple<
    // T
    Time,
    // K
    Divided<LinearVelocity, Voltage>,
    // Ka
    KaUnits<T>,
    // Kp
    Divided<Voltage, LinearVelocity>,
    // Ki
    Divided<Voltage, Length>> {
    // of form < voltage, -velocity >
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(data.size() - 1, 2);
    // of form < accel >
    Eigen::VectorXd b = Eigen::VectorXd::Zero(data.size() - 1);

    for (size_t i = 1; i < data.size(); i++) {
        DataT& last = data[i - 1];
        DataT& curr = data[i];

        AccelUnit accel = (curr.velocity - last.velocity) / delta_time;

        A(i - 1, 0) = curr.voltage.internal();
        A(i - 1, 1) = -curr.velocity.internal();

        b(i - 1) = accel.internal();
    }

    Eigen::VectorXd solution =
      A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    Divided<LinearAcceleration, Voltage> h { solution(0) };
    Frequency g { solution(1) };

    Time _T = 1 / g;
    Divided<LinearVelocity, Voltage> K = h / g;

    auto [Ka, Kp, Ki] = calculate_ka_kp_ki_from_TK(_T, K, lambda_factor);

    return { _T, K, Ka, Kp, Ki };
}

// fits data using different model:
// velocity_next = a1 * velocity + a2 * voltage
//
template<typename T>
auto MotorGroupSysid<T>::fit_ka_kp_ki_data_second_model(const VectorDataT& data,
                                                        Time delta_time,
                                                        double lambda_factor)
  -> std::tuple<
    // T
    Time,
    // K
    Divided<LinearVelocity, Voltage>,
    // Ka
    KaUnits<T>,
    // Kp
    Divided<Voltage, LinearVelocity>,
    // Ki
    Divided<Voltage, Length>> {
    // of form < velocity, voltage >
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(data.size() - 1, 2);
    // of form < next velocity >
    Eigen::VectorXd b = Eigen::VectorXd::Zero(data.size() - 1);

    for (size_t i = 1; i < data.size(); i++) {
        DataT& prev = data[i - 1];
        DataT& next = data[i];

        A(i - 1, 0) = prev.velocity.internal();
        A(i - 1, 1) = prev.voltage.internal();

        b(i - 1) = next.velocity.internal();
    }

    Eigen::VectorXd solution =
      A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    Number a1 { solution(0) };
    Divided<LinearVelocity, Voltage> a2 { solution(1) };

    Time _T = -delta_time / std::log(a1);
    Divided<LinearVelocity, Voltage> K = -a2 / (a1 - 1);

    auto [Ka, Kp, Ki] = calculate_ka_kp_ki_from_TK(_T, K, lambda_factor);

    return { _T, K, Ka, Kp, Ki };
}

// averages results from both models. This seems to produce really good
// results as both deviate in opposite directions
template<typename T>
auto MotorGroupSysid<T>::fit_ka_kp_ki_data_both_models(const VectorDataT& data,
                                                       Time delta_time,
                                                       double lambda_factor)
  -> std::tuple<
    // T
    Time,
    // K
    Divided<LinearVelocity, Voltage>,
    // Ka
    KaUnits<T>,
    // Kp
    Divided<Voltage, LinearVelocity>,
    // Ki
    Divided<Voltage, Length>> {
    auto [first_T, first_K, first_Ka, first_Kp, first_Ki] =
      fit_ka_kp_ki_data_first_model(data, delta_time, lambda_factor);
    auto [second_T, second_K, second_Ka, second_Kp, second_Ki] =
      fit_ka_kp_ki_data_second_model(data, delta_time, lambda_factor);

    Time avg_T = (first_T + second_T) / 2.0;
    Divided<LinearVelocity, Voltage> avg_K = (first_K + second_K) / 2.0;

    auto [avg_Ka, avg_Kp, avg_Ki] =
      calculate_ka_kp_ki_from_TK(avg_T, avg_K, lambda_factor);

    return { avg_T, avg_K, avg_Ka, avg_Kp, avg_Ki };
}

// fits data specifically only for ka with model:
// accel * ka = voltage - velocity * kv - sgn(velocity) * ks
template<typename T>
KaUnits<T> MotorGroupSysid<T>::fit_ka_data(const VectorDataT& data,
                                           Time delta_time,
                                           KvUnits<T> kv,
                                           KsUnits ks) {
    // of form < accel >
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(data.size() - 1, 1);
    // of form < voltage - velocity * kv - sgn(velocity) * ks >
    Eigen::VectorXd b = Eigen::VectorXd::Zero(data.size() - 1);

    for (size_t i = 1; i < data.size(); i++) {
        DataT& curr = data[i];
        DataT& last = data[i - 1];

        AccelUnit accel = (curr.velocity - last.velocity) / delta_time;
        Voltage accel_voltage =
          (curr.voltage - curr.velocity * kv - units::sgn(curr.velocity) * ks);

        A(i - 1, 0) = accel.internal();
        b(i - 1) = accel_voltage.internal();
    }

    Eigen::VectorXd solution =
      A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    KaUnits<T> ka { solution(0) };
    return { ka };
}

// print data in desmos-friendly format

template<typename T>
void print_data_as_latex(const std::vector<MotorSysidData<T>>& data) {
    std::cout << "\\left[";
    for (int i = 0; i < data.size(); i++) {
        std::cout << "\\left(" << data[i].voltage.internal() << ","
                  << data[i].velocity.convert(mps) << "\\right)";
        // doesn't print comma for last point
        if (i < data.size() - 1) std::cout << ",";
    }
    std::cout << "\\right]" << std::endl;
}

template struct MotorSysidData<LinearVelocity>;
template struct MotorSysidData<AngularVelocity>;
template class MotorGroupSysid<LinearVelocity>;
template class MotorGroupSysid<AngularVelocity>;

// differential sysid methods

// gathers velocity data from robot by moving voltage_commands
// does not collect actual voltage data, but rather commanded voltage
DifferentialSysidData DifferentialSysid::createData(
  std::vector<DifferentialSysIdVoltageCommands> voltage_commands,
  DifferentialDrivetrain& drivetrain,
  Time delta_time) {

    DifferentialSysidData data;

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

                data.left.emplace_back(left_velocity, left_voltage);
                data.right.emplace_back(right_velocity, right_voltage);
            }

            pros::c::task_delay_until(&prev_time, int_delta_time);
        }
    }
    return data;
}

DifferentialSysidData DifferentialSysid::gather_kv_ks_data(
  std::vector<DifferentialSysIdVoltageCommands> voltage_commands,
  DifferentialDrivetrain& drivetrain,
  Time delta_time,
  Time steady_state_time) {
    DifferentialSysidData data;

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
                left_averageVoltages += left_voltage;

                right_averageVelocities += curr_right_vel;
                right_averageVoltages += right_voltage;

                samples++;
            }

            pros::c::task_delay_until(&prev_time, int_delta_time);
        }

        left_averageVelocities /= samples;
        left_averageVoltages /= samples;

        right_averageVelocities /= samples;
        right_averageVoltages /= samples;

        if (record) {
            data.left.emplace_back(left_averageVelocities,
                                   left_averageVoltages);
            data.right.emplace_back(right_averageVelocities,
                                    right_averageVoltages);
        }
    }

    return data;
}

DifferentialSysidData DifferentialSysid::calculate_kv_ks(
  std::vector<DifferentialSysIdVoltageCommands> voltage_commands,
  DifferentialDrivetrain& drivetrain,
  Time delta_time,
  Time steady_state_time) {
    // figure out kv and ks from this data

    DifferentialSysidData data = gather_kv_ks_data(voltage_commands,
                                                   drivetrain,
                                                   delta_time,
                                                   steady_state_time);

    auto [left_kv, left_ks] =
      MotorGroupSysidLinearVelocity::fit_kv_ks_data(data.left);
    auto [right_kv, right_ks] =
      MotorGroupSysidLinearVelocity::fit_kv_ks_data(data.right);

    std::cout << "left: " << std::endl;
    std::cout << "kv: " << left_kv << ", ks: " << left_ks << std::endl;

    std::cout << "right: " << std::endl;
    std::cout << "kv: " << right_kv << ", ks: " << right_ks << std::endl;

    std::cout << std::endl;
    std::cout << "copiable data:\n\n";

    // clang-format off
    std::cout << ".left_Kv = " << left_kv.internal() << " * volt / mps,\n"
              << ".left_Ks = " << left_ks.internal() << " * volt,\n"
              << "\n"
              << ".right_Kv = " << right_kv.internal() << " * volt / mps,\n"
              << ".right_Ks = " << right_ks.internal() << " * volt,\n"
              << std::endl;
    // clang-format on

    return data;
}

DifferentialSysidData DifferentialSysid::calculate_ka(
  std::vector<DifferentialSysIdVoltageCommands> voltage_commands,
  DifferentialDrivetrain& drivetrain,
  KvUnits<LinearVelocity> left_kv,
  KsUnits left_ks,
  KvUnits<LinearVelocity> right_kv,
  KsUnits right_ks,
  Time delta_time) {

    DifferentialSysidData data =
      createData(voltage_commands, drivetrain, delta_time);

    // figure out kv and ks from this data

    auto left_ka = MotorGroupSysidLinearVelocity::fit_ka_data(data.left,
                                                              delta_time,
                                                              left_kv,
                                                              left_ks);
    auto right_ka = MotorGroupSysidLinearVelocity::fit_ka_data(data.right,
                                                               delta_time,
                                                               right_kv,
                                                               right_ks);

    std::cout << "left: " << std::endl;
    std::cout << "ka: " << left_ka << std::endl;

    std::cout << "right: " << std::endl;
    std::cout << "ka: " << right_ka << std::endl;

    return data;
}

void DifferentialSysid::printData(DifferentialSysidData data, Time delta_time) {
    std::cout << "delta time of " << to_msec(delta_time) << " msec\n";

    std::cout << "left motor data (voltage, velocity):\n";
    print_data_as_latex(data.left);

    std::cout << "right motor data(voltage, velocity):\n ";
    print_data_as_latex(data.right);
}

DifferentialSysidData DifferentialSysid::calculate_ka_kp_ki_fopdt(
  DifferentialSysIdVoltageCommands voltage_command,
  DifferentialDrivetrain& drivetrain,
  Time delta_time,
  double lambda_factor) {

    DifferentialSysidData data = createData(
      std::vector<DifferentialSysIdVoltageCommands> { voltage_command },
      drivetrain,
      delta_time);

    auto [left_T, left_K, left_ka, left_kp, left_ki] =
      MotorGroupSysidLinearVelocity::fit_ka_kp_ki_data_both_models(
        data.left,
        delta_time,
        lambda_factor);
    auto [right_T, right_K, right_ka, right_kp, right_ki] =
      MotorGroupSysidLinearVelocity::fit_ka_kp_ki_data_both_models(
        data.right,
        delta_time,
        lambda_factor);

    std::cout << "left: " << std::endl;
    std::cout << "T: " << left_T << ", K: " << left_K
              << ", lambda factor: " << lambda_factor << std::endl;
    std::cout << "ka: " << left_ka << ", kp: " << left_kp << ", ki: " << left_ki
              << std::endl;

    std::cout << "right: " << std::endl;
    std::cout << "T: " << right_T << ", K: " << right_K
              << ", lambda factor: " << lambda_factor << std::endl;
    std::cout << "ka: " << right_ka << ", kp: " << right_kp
              << ", ki: " << right_ki << std::endl;

    std::cout << std::endl;
    std::cout << "copiable data:\n\n";

    // clang-format off
    std::cout << ".left_Ka = " << left_ka.internal() << " * volt / mps2,\n"
              << "// lambda factor: " << lambda_factor <<  "\n"
              << ".left_Kp = " << left_kp.internal() << " * volt / mps,\n"
              << ".left_Ki = " << left_ki.internal() << " * volt / m,\n"
              << "\n"
              << ".right_Ka = " << right_ka.internal() << " * volt / mps2,\n"
              << "// lambda factor: " << lambda_factor <<  "\n"
              << ".right_Kp = " << right_kp.internal() << " * volt / mps,\n"
              << ".right_Ki = " << right_ki.internal() << " * volt / m," << std::endl;
    // clang-format on

    return data;
}

} // namespace lyfast
} // namespace blazing
