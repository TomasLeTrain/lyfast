#include "lyfast/system_identification.hpp"
#include "pros/abstract_motor.hpp"
#include "units/Angle.hpp"

namespace blazing {
namespace lyfast {

std::vector<OLS_data>
createData(std::vector<SysIdVoltageCommands> voltage_commands,
           pros::MotorGroup* left_motors,
           pros::MotorGroup* right_motors,
           Length wheel_diameter,
           AngularVelocity final_rpm) {
    std::vector<OLS_data> data;

    uint32_t delta_time = 10;

    for (auto [left_voltage, right_voltage, target_time] : voltage_commands) {
        left_motors->move_voltage(to_mvolt(12 * left_voltage));
        right_motors->move_voltage(to_mvolt(12 * right_voltage));

        auto start_time = blazing::now();
        uint32_t prev_time;

        auto get_group_velocity =
          [&](pros::MotorGroup* motor_group) -> LinearVelocity {
            AngularVelocity average_rpm = 0_rpm;

            for (size_t motor_i = 0; motor_i < motor_group->size(); motor_i++) {
                double velocity = motor_group->get_actual_velocity(motor_i);
                pros::MotorGears encoder_units =
                  motor_group->get_gearing(motor_i);
                AngularVelocity start_rpm;

                switch (encoder_units) {
                    case pros::MotorGears::blue: start_rpm = 600_rpm;
                    case pros::MotorGears::green: start_rpm = 200_rpm;
                    case pros::MotorGears::red: start_rpm = 100_rpm;
                    default: 200_rpm;
                }

                AngularVelocity actual_rpm =
                  (velocity * rpm) * final_rpm / start_rpm;
                average_rpm += actual_rpm;
            }

            average_rpm /= motor_group->size();

            LinearVelocity velocity =
              average_rpm * (wheel_diameter * 2 * M_PI) / rot;

            return velocity;
        };

        while (!timeoutDone(target_time, start_time)) {
            prev_time = pros::millis();

            LinearVelocity left_velocity = get_group_velocity(left_motors);
            LinearVelocity right_velocity = get_group_velocity(right_motors);

            data.emplace_back(left_velocity,
                              right_velocity,
                              left_voltage,
                              right_voltage);

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
