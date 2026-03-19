#include "lyfast/sysid/system_identification.hpp"
#include "Eigen/Dense"
#include "blazing/drivetrains/differential.hpp"
#include "blazing/latex_utils.hpp"
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
namespace sysid {

// explicit instantiations
template struct SysidEntry<LinearVelocity>;
template struct SysidEntry<AngularVelocity>;
template class MotorGroupUtils<LinearVelocity>;
template class MotorGroupUtils<AngularVelocity>;

// gathers velocity data from robot by moving voltage_commands
template<typename T>
MotorGroupUtils<T>::VectorDataT
MotorGroupUtils<T>::generateData(std::vector<VoltageCommand> voltage_commands,
                                 pros::MotorGroup* motor_group,
                                 std::function<T()> velocity_func,
                                 std::function<Voltage(Voltage)> voltage_func,
                                 std::optional<Time> steady_state_time,
                                 Time delta_time) {

    uint32_t int_delta_time = std::lround(to_msec(delta_time));
    VectorDataT data;

    for (auto [voltage, target_time, record] : voltage_commands) {
        motor_group->move_voltage(12 * to_mvolt(voltage));

        auto start_time = blazing::now();
        uint32_t prev_time = pros::millis();

        while (!timeoutDone(target_time, start_time)) {
            // time at which we start to record data
            FTime threshold_time = 0_Fsec;

            if (steady_state_time)
                threshold_time =
                  units::max(0_Fsec, target_time - steady_state_time.value());

            if (timeoutDone(threshold_time, start_time) && record) {
                T velocity = velocity_func();

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
MotorGroupUtils<T>::fit_kv_ks_data(const VectorDataT& data) {
    // of form < velocity, sgn(velocity) >
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(data.size(), 2);
    // of form < Voltage >
    Eigen::VectorXf b = Eigen::VectorXf::Zero(data.size());

    for (size_t i = 0; i < data.size(); i++) {
        const DataT& curr = data[i];

        A(i, 0) = curr.velocity.internal();
        A(i, 1) = units::sgn(curr.velocity);

        b(i) = curr.voltage.internal();
    }

    Eigen::VectorXf solution =
      A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    KvUnits<T> kv { solution(0) };
    KsUnits ks { solution(1) };
    return { kv, ks };
}

// calculates ka/kp/ki from T and K constants and lambda factor
template<typename T>
auto MotorGroupUtils<T>::calculate_ka_kp_ki_from_TK(Time _T,
                                                    Divided<T, Voltage> K,
                                                    double lambda_factor)
  -> std::tuple<
    // Ka
    KaUnits<T>,
    // Kp
    KpUnits<T>,
    // Ki
    KiUnits<T>> {

    KaUnits<T> Ka = _T / K;

    Time lambda = lambda_factor * _T;

    KpUnits<T> Kp = _T / (K * lambda);
    KiUnits<T> Ki = Kp / _T;

    return { Ka, Kp, Ki };
}

// fits various values data using model:
// accel = voltage * h - velocity * g
template<typename T>
MotorGroupUtils<T>::ka_ki_kp_dataT
MotorGroupUtils<T>::fit_ka_kp_ki_data_first_model(const VectorDataT& data,
                                                  Time delta_time,
                                                  double lambda_factor) {
    // of form < voltage, -velocity >
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(data.size() - 1, 2);
    // of form < accel >
    Eigen::VectorXf b = Eigen::VectorXf::Zero(data.size() - 1);

    for (size_t i = 0; i < data.size() - 1; i++) {
        const DataT& curr = data[i];
        const DataT& next = data[i + 1];

        AccelUnit accel = (next.velocity - curr.velocity) / delta_time;

        A(i, 0) = curr.voltage.internal();
        A(i, 1) = -curr.velocity.internal();

        b(i) = accel.internal();
    }

    Eigen::VectorXf solution =
      A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    Divided<AccelUnit, Voltage> h { solution(0) };
    Frequency g { solution(1) };

    Time _T = 1 / g;
    Divided<T, Voltage> K = h / g;

    auto [Ka, Kp, Ki] = calculate_ka_kp_ki_from_TK(_T, K, lambda_factor);

    return { _T, K, Ka, Kp, Ki };
}

// fits data using different model:
// velocity_next = a1 * velocity + a2 * voltage
//
template<typename T>
MotorGroupUtils<T>::ka_ki_kp_dataT
MotorGroupUtils<T>::fit_ka_kp_ki_data_second_model(const VectorDataT& data,
                                                   Time delta_time,
                                                   double lambda_factor) {
    // of form < velocity, voltage >
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(data.size() - 1, 2);
    // of form < next velocity >
    Eigen::VectorXf b = Eigen::VectorXf::Zero(data.size() - 1);

    for (size_t i = 0; i < data.size() - 1; i++) {
        const DataT& curr = data[i];
        const DataT& next = data[i + 1];

        A(i, 0) = curr.velocity.internal();
        A(i, 1) = curr.voltage.internal();

        b(i) = next.velocity.internal();
    }

    Eigen::VectorXf solution =
      A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    Number a1 { solution(0) };
    Divided<T, Voltage> a2 { solution(1) };

    Time _T = -delta_time / std::log(a1);
    Divided<T, Voltage> K = -a2 / (a1 - 1);

    auto [Ka, Kp, Ki] = calculate_ka_kp_ki_from_TK(_T, K, lambda_factor);

    return { _T, K, Ka, Kp, Ki };
}

// averages results from both models. This seems to produce really good
// results as both deviate in opposite directions
template<typename T>
MotorGroupUtils<T>::ka_ki_kp_dataT
MotorGroupUtils<T>::fit_ka_kp_ki_data_both_models(const VectorDataT& data,
                                                  Time delta_time,
                                                  double lambda_factor) {
    auto [first_T, first_K, first_Ka, first_Kp, first_Ki] =
      fit_ka_kp_ki_data_first_model(data, delta_time, lambda_factor);
    auto [second_T, second_K, second_Ka, second_Kp, second_Ki] =
      fit_ka_kp_ki_data_second_model(data, delta_time, lambda_factor);

    Time avg_T = (first_T + second_T) / 2.0;
    Divided<T, Voltage> avg_K = (first_K + second_K) / 2.0;

    auto [avg_Ka, avg_Kp, avg_Ki] =
      calculate_ka_kp_ki_from_TK(avg_T, avg_K, lambda_factor);

    return { avg_T, avg_K, avg_Ka, avg_Kp, avg_Ki };
}

// fits data specifically only for ka with model:
// accel * ka = voltage - velocity * kv - sgn(velocity) * ks
template<typename T>
KaUnits<T> MotorGroupUtils<T>::fit_ka_data(const VectorDataT& data,
                                           Time delta_time,
                                           KvUnits<T> kv,
                                           KsUnits ks) {
    // of form < accel >
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(data.size() - 1, 1);
    // of form < voltage - velocity * kv - sgn(velocity) * ks >
    Eigen::VectorXf b = Eigen::VectorXf::Zero(data.size() - 1);

    for (size_t i = 0; i < data.size() - 1; i++) {
        const DataT& curr = data[i];
        const DataT& next = data[i + 1];

        AccelUnit accel = (next.velocity - curr.velocity) / delta_time;
        Voltage accel_voltage =
          (curr.voltage - curr.velocity * kv - units::sgn(curr.velocity) * ks);

        A(i, 0) = accel.internal();
        b(i) = accel_voltage.internal();
    }

    Eigen::VectorXf solution =
      A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

    KaUnits<T> ka { solution(0) };
    return { ka };
}

// print data in desmos-friendly format - prints in VelUnit units
template<typename T>
void MotorGroupUtils<T>::printDataAsLatex(const VectorDataT& data) {
    printPairListAsLatex(data.size(), [&](size_t i) -> std::pair<float, float> {
        return { data[i].voltage.internal(),
                 data[i].velocity.convert(T { 1 }) };
    });
}

// differential sysid methods
// return calculated [left,right] kv/ks. Also prints values
std::pair<std::pair<KvUnits<LinearVelocity>, KsUnits>,
          std::pair<KvUnits<LinearVelocity>, KsUnits>>
DifferentialUtils::calculate_kv_ks(const DifferentialData& data) {
    // figure out kv and ks from the data
    auto [left_kv, left_ks] = LinearMotorGroupUtils::fit_kv_ks_data(data.left);
    auto [right_kv, right_ks] =
      LinearMotorGroupUtils::fit_kv_ks_data(data.right);

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
    return {
        { left_kv,  left_ks  },
        { right_kv, right_ks }
    };
}

std::pair<KaUnits<LinearVelocity>, KaUnits<LinearVelocity>>
DifferentialUtils::calculate_ka(const DifferentialData& data,
                                KvUnits<LinearVelocity> left_kv,
                                KsUnits left_ks,
                                KvUnits<LinearVelocity> right_kv,
                                KsUnits right_ks,
                                Time delta_time) {
    // figure out kv and ks from this data

    auto left_ka = LinearMotorGroupUtils::fit_ka_data(data.left,
                                                      delta_time,
                                                      left_kv,
                                                      left_ks);
    auto right_ka = LinearMotorGroupUtils::fit_ka_data(data.right,
                                                       delta_time,
                                                       right_kv,
                                                       right_ks);

    std::cout << "left: " << std::endl;
    std::cout << "ka: " << left_ka << std::endl;

    std::cout << "right: " << std::endl;
    std::cout << "ka: " << right_ka << std::endl;

    return { left_ka, right_ka };
}

std::pair<KaUnits<LinearVelocity>, KaUnits<LinearVelocity>>
calculate_ka_kp_ki_fopdt(const DifferentialData& data,
                         Time delta_time,
                         double lambda_factor) {
    auto [left_T, left_K, left_ka, left_kp, left_ki] =
      LinearMotorGroupUtils::fit_ka_kp_ki_data_both_models(data.left,
                                                           delta_time,
                                                           lambda_factor);
    auto [right_T, right_K, right_ka, right_kp, right_ki] =
      LinearMotorGroupUtils::fit_ka_kp_ki_data_both_models(data.right,
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

    return { left_ka, right_ka };
}

void DifferentialUtils::printData(const DifferentialData& data,
                                  Time delta_time) {
    std::cout << "delta time of " << to_msec(delta_time) << " msec\n";

    std::cout << "left motor data (voltage, velocity):\n";
    LinearMotorGroupUtils::printDataAsLatex(data.left);

    std::cout << "right motor data(voltage, velocity):\n ";
    LinearMotorGroupUtils::printDataAsLatex(data.right);
}

} // namespace sysid
} // namespace lyfast
} // namespace blazing
