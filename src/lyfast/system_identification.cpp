#include "lyfast/system_identification.hpp"
#include "Eigen/Dense"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

// __attribute__((used))
std::vector<OLS_data>
calculate_kv_ks(std::vector<SysIdVoltageCommands> voltage_commands,
                pros::MotorGroup* left_motors,
                pros::MotorGroup* right_motors,
                Length wheel_diameter,
                AngularVelocity final_rpm) {
    std::vector<OLS_data> data;

    for (auto [left_voltage, right_voltage, target_time, record] :
         voltage_commands) {
        left_motors->move_voltage(to_mvolt(12 * left_voltage));
        right_motors->move_voltage(to_mvolt(12 * right_voltage));

        auto start_time = blazing::now();
        uint32_t prev_time;
        uint32_t delta_time = 10;

        auto get_group_velocity =
          [&](pros::MotorGroup* motor_group) -> LinearVelocity {
            AngularVelocity average_rpm = 0_rpm;

            for (std::int8_t motor_i = 0; motor_i < motor_group->size();
                 motor_i++) {
                double velocity = motor_group->get_actual_velocity(motor_i);
                pros::MotorGears encoder_units =
                  motor_group->get_gearing(motor_i);
                AngularVelocity start_rpm;

                switch (encoder_units) {
                    case pros::MotorGears::blue: start_rpm = 600_rpm; break;
                    case pros::MotorGears::green: start_rpm = 200_rpm; break;
                    case pros::MotorGears::red: start_rpm = 100_rpm; break;
                    default: 200_rpm; break;
                }

                AngularVelocity actual_rpm =
                  (velocity * rpm) * final_rpm / start_rpm;

                average_rpm += actual_rpm;
            }

            average_rpm /= motor_group->size();

            LinearVelocity velocity =
              average_rpm * (wheel_diameter * M_PI) / rot;

            return velocity;
        };

        auto get_voltage = [](pros::MotorGroup* motors) -> Voltage {
            Voltage result = 0_volt;
            for (auto voltage : motors->get_voltage_all()) {
                result += from_mvolt(voltage) / 12;
            }
            result /= motors->size();
            return result;
        };

        units::V2Velocity averageVelocities;
        units::Vector2D<Voltage> averageVoltages;
        int samples = 0;

        while (!timeoutDone(target_time, start_time)) {
            // amount of time to measure the steady state
            Time steady_state_time = 200_msec;

            if (timeoutDone(units::max(0_sec, target_time - steady_state_time),
                            start_time)) {
                prev_time = pros::millis();

                averageVelocities.x += get_group_velocity(left_motors);
                averageVoltages.x += get_voltage(left_motors);

                averageVelocities.y += get_group_velocity(right_motors);
                averageVoltages.y += get_voltage(right_motors);
                samples++;
            }

            pros::c::task_delay_until(&prev_time, delta_time);
        }
        averageVelocities /= samples;
        averageVoltages /= samples;

        // x for left, y for right
        data.emplace_back(averageVelocities.x,
                          averageVelocities.y,
                          averageVoltages.x,
                          averageVoltages.y);
    }

    // figure out kv and ks from this data

    auto fitData = [&](bool left) -> std::pair<KvUnits, KsUnits> {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(data.size(), 2);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(data.size());

        for (int i = 0; i < data.size(); i++) {
            if (left) {
                A(i, 0) = data[i].left_velocity.internal();
                A(i, 1) = units::sgn(data[i].left_velocity);
                b(i) = data[i].left_voltage.internal();
            } else {
                A(i, 0) = data[i].right_velocity.internal();
                A(i, 1) = units::sgn(data[i].right_velocity);
                b(i) = data[i].right_voltage.internal();
            }
        }

        Eigen::VectorXd solution =
          A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

        KvUnits kv { solution(0) };
        KsUnits ks { solution(1) };
        return { kv, ks };
    };

    auto left_kv_ks = fitData(true);
    auto right_kv_ks = fitData(false);

    std::cout << "left: " << std::endl;
    std::cout << "kv: " << left_kv_ks.first << ", ks: " << left_kv_ks.second
              << std::endl;

    std::cout << "right: " << std::endl;
    std::cout << "kv: " << right_kv_ks.first << ", ks: " << right_kv_ks.second
              << std::endl;

    return data;
}

std::vector<OLS_data>
calculate_ka(std::vector<SysIdVoltageCommands> voltage_commands,
             pros::MotorGroup* left_motors,
             pros::MotorGroup* right_motors,
             Length wheel_diameter,
             AngularVelocity final_rpm,
             KvUnits left_kv,
             KsUnits left_ks,
             KvUnits right_kv,
             KsUnits right_ks) {
    std::vector<OLS_data> data;
    uint32_t delta_time = 10;

    for (auto [left_voltage, right_voltage, target_time, record] :
         voltage_commands) {
        left_motors->move_voltage(to_mvolt(12 * left_voltage));
        right_motors->move_voltage(to_mvolt(12 * right_voltage));

        auto start_time = blazing::now();
        uint32_t prev_time;

        auto get_group_velocity =
          [&](pros::MotorGroup* motor_group) -> LinearVelocity {
            AngularVelocity average_rpm = 0_rpm;

            for (std::int8_t motor_i = 0; motor_i < motor_group->size();
                 motor_i++) {
                double velocity = motor_group->get_actual_velocity(motor_i);
                pros::MotorGears encoder_units =
                  motor_group->get_gearing(motor_i);
                AngularVelocity start_rpm;

                switch (encoder_units) {
                    case pros::MotorGears::blue: start_rpm = 600_rpm; break;
                    case pros::MotorGears::green: start_rpm = 200_rpm; break;
                    case pros::MotorGears::red: start_rpm = 100_rpm; break;
                    default: 200_rpm; break;
                }

                AngularVelocity actual_rpm =
                  (velocity * rpm) * final_rpm / start_rpm;

                average_rpm += actual_rpm;
            }

            average_rpm /= motor_group->size();

            LinearVelocity velocity =
              average_rpm * (wheel_diameter * M_PI) / rot;

            return velocity;
        };

        auto get_voltage = [](pros::MotorGroup* motors) -> Voltage {
            Voltage result = 0_volt;
            for (auto voltage : motors->get_voltage_all()) {
                result += from_mvolt(voltage) / 12;
            }
            result /= motors->size();
            return result;
        };

        while (!timeoutDone(target_time, start_time)) {
            // amount of time to measure the steady state
            auto left_velocity = get_group_velocity(left_motors);
            auto left_voltage = get_voltage(left_motors);

            auto right_velocity = get_group_velocity(right_motors);
            auto right_voltage = get_voltage(right_motors);

            data.emplace_back(left_velocity,
                              right_velocity,
                              left_voltage,
                              right_voltage);

            pros::c::task_delay_until(&prev_time, delta_time);
        }
        std::cout << "finished command" << std::endl;
    }

    // figure out kv and ks from this data

    auto fitData = [&](bool left) -> KaUnits {
        std::cout << "data size: " << data.size() << std::endl;

        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(data.size() - 1, 1);
        std::cout << "defining b" << std::endl;
        Eigen::VectorXd b = Eigen::VectorXd::Zero(data.size() - 1);

        std::cout << "filling matrices solution" << std::endl;

        for (int i = 1; i < data.size(); i++) {
            if (left) {
                auto left_accel =
                  (data[i].left_velocity - data[i - 1].left_velocity) /
                  from_msec(delta_time);

                A(i - 1, 0) = left_accel.internal();
                b(i - 1) =
                  (data[i].left_voltage - data[i].left_velocity * left_kv +
                   units::sgn(data[i].left_velocity) * left_ks)
                    .internal();
            } else {
                auto right_accel =
                  (data[i].right_velocity - data[i - 1].right_velocity) /
                  from_msec(delta_time);

                A(i - 1, 0) = right_accel.internal();
                b(i - 1) =
                  (data[i].right_voltage - data[i].right_velocity * right_kv +
                   units::sgn(data[i].right_velocity) * right_ks)
                    .internal();
            }
        }
        std::cout << "finding solution" << std::endl;

        Eigen::VectorXd solution =
          A.bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);

        std::cout << "solution found" << std::endl;

        KaUnits ka { solution(0) };
        return { ka };
    };

    auto left_ka = fitData(true);
    auto right_ka = fitData(false);

    std::cout << "left: " << std::endl;
    std::cout << "ka: " << left_ka << std::endl;

    std::cout << "right: " << std::endl;
    std::cout << "ka: " << right_ka << std::endl;

    return data;
}

std::vector<OLS_data>
createData(std::vector<SysIdVoltageCommands> voltage_commands,
           pros::MotorGroup* left_motors,
           pros::MotorGroup* right_motors,
           Length wheel_diameter,
           AngularVelocity final_rpm) {
    std::vector<OLS_data> data;

    uint32_t delta_time = 10;

    for (auto [left_voltage, right_voltage, target_time, record] :
         voltage_commands) {
        left_motors->move_voltage(to_mvolt(12 * left_voltage));
        right_motors->move_voltage(to_mvolt(12 * right_voltage));

        auto start_time = blazing::now();
        uint32_t prev_time;

        auto get_group_velocity =
          [&](pros::MotorGroup* motor_group) -> LinearVelocity {
            AngularVelocity average_rpm = 0_rpm;

            for (std::int8_t motor_i = 0; motor_i < motor_group->size();
                 motor_i++) {
                double velocity = motor_group->get_actual_velocity(motor_i);
                pros::MotorGears encoder_units =
                  motor_group->get_gearing(motor_i);
                AngularVelocity start_rpm;

                switch (encoder_units) {
                    case pros::MotorGears::blue: start_rpm = 600_rpm; break;
                    case pros::MotorGears::green: start_rpm = 200_rpm; break;
                    case pros::MotorGears::red: start_rpm = 100_rpm; break;
                    default: 200_rpm; break;
                }

                AngularVelocity actual_rpm =
                  (velocity * rpm) * final_rpm / start_rpm;

                average_rpm += actual_rpm;
            }

            average_rpm /= motor_group->size();

            LinearVelocity velocity =
              average_rpm * (wheel_diameter * M_PI) / rot;

            return velocity;

            // debug calculations by outputting average rpm instead
            // LinearVelocity fake_velocity = average_rpm.convert(rpm) * mps;
            //
            // return fake_velocity;
        };

        auto get_voltage = [](pros::MotorGroup* motors) -> Voltage {
            Voltage result = 0_volt;
            for (auto voltage : motors->get_voltage_all()) {
                result += from_mvolt(voltage) / 12;
            }
            result /= motors->size();
            return result;
        };

        while (!timeoutDone(target_time, start_time)) {
            prev_time = pros::millis();

            LinearVelocity left_velocity = get_group_velocity(left_motors);
            LinearVelocity right_velocity = get_group_velocity(right_motors);
            Voltage left_volt = get_voltage(left_motors);
            Voltage right_volt = get_voltage(right_motors);

            if (record)
                data.emplace_back(left_velocity,
                                  right_velocity,
                                  left_volt,
                                  right_volt);

            pros::c::task_delay_until(&prev_time, delta_time);
        }
    }
    return data;
}

void printData(std::vector<OLS_data> data) {
    int delta_time = 10;
    std::println("delta time of {}\n", delta_time);

    std::println("left motor data (voltage, velocity):");
    std::print("\\left[");
    for (auto datapoint : data) {
        std::print("\\left({},{}\\right),",
                   datapoint.left_voltage.internal(),
                   datapoint.left_velocity.convert(mps));
    }
    std::println("\\right]");

    std::println("right motor data (voltage, velocity):");
    std::print("\\left[");
    for (auto datapoint : data) {
        std::print("\\left({},{}\\right),",
                   datapoint.right_voltage.internal(),
                   datapoint.right_velocity.convert(mps));
    }
    std::println("\\right]");
}

} // namespace lyfast
} // namespace blazing
