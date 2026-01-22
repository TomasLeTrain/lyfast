#include "lyfast/system_identification.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {
void calculate_kv_ks(std::vector<SysIdVoltageCommands> voltage_commands,
                     pros::MotorGroup* left_motors,
                     pros::MotorGroup* right_motors,
                     Length wheel_diameter,
                     AngularVelocity final_rpm) {
    KvUnits kv { 0 };
    KsUnits ks { 0 };

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
            Time steady_state_time = 300_msec;

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

    std::cout << "left: " << std::endl;
    std::cout << "kv: " << ", ks ";
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
