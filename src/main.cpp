#include "main.h"
#include "blazing/api.hpp"
#include "blazing/utils.hpp"
#include "lyfast/api.hpp"
#include "lyfast/system_identification.hpp"
#include "lyfast/vel_controller.hpp"
#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/motor_group.hpp"
#include "pros/optical.h"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <iostream>
#include <mutex>
#include <numeric>
#include <utility>

void disabled() {}

void competition_initialize() {}

void autonomous() {}

class ScaledIMU : public pros::IMU {
  public:
    ScaledIMU(int port, double scalar = 1.0)
        : pros::IMU(port),
          m_scalar(scalar),
          m_port(port) {}

    ScaledIMU(const pros::IMU& other, double scalar = 1.0)
        : pros::IMU(other),
          m_scalar(scalar),
          m_port(other.get_port()) {}

    int32_t reset(bool blocking = false) {
        std::lock_guard lock(m_mutex);

        m_offset = 0;
        return pros::IMU::reset(blocking);
    }

    virtual double get_rotation() const {
        std::lock_guard lock(m_mutex);

        double raw = pros::c::imu_get_rotation(m_port);
        if (raw == INFINITY) return INFINITY;
        return raw * m_scalar + m_offset;
    }

    virtual int set_rotation(double new_rotation) {
        std::lock_guard lock(m_mutex);

        double curr_raw = this->get_rotation();
        if (curr_raw == INFINITY) return INT32_MAX;

        m_offset += new_rotation - curr_raw;
        return 0;
    }

    // is this required?
    // virtual pros::imu_gyro_s_t get_gyro_rate() const {
    //     std::lock_guard lock(m_mutex);
    //
    //     pros::imu_gyro_s_t raw = pros::c::imu_get_gyro_rate(m_port);
    //     pros::imu_gyro_s_t scaled = { raw.x, raw.y, raw.z * m_scalar };
    //
    //     return scaled;
    // }

  private:
    const double m_scalar;
    int m_port;

    mutable pros::Mutex m_mutex;

    double m_offset = 0;
};

// clang-format off
// motor groups


int8_t left_front = 3;
int8_t left_middle = -1;
int8_t left_back = -15;

int8_t right_front = -13;
int8_t right_middle = 14;
int8_t right_back = 12;

bool vexmaps_logging_enabled = false;

pros::MotorGroup left_motors({ left_front, left_middle, left_back }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::MotorGroup right_motors({ right_front, right_middle, right_back }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
// clang-format on

ScaledIMU imu(15, (360.0 + 3.8) / 360.0);
pros::Controller master(pros::E_CONTROLLER_MASTER);

// odom rotation sensors
pros::Rotation forwards_odom_rotation(4);
pros::Rotation sideways_odom_rotation(3);

using namespace blazing;

// tracker stuff
DifferentialDrivetrain drivetrain(&left_motors, &right_motors);

Length track_width = 10.5_in;
Length wheel_diameter = 3.25_in;
AngularVelocity final_rpm = 450_rpm;

ForwardsTracker
  left_motor_tracker(&left_motors, -track_width / 2, wheel_diameter, final_rpm);

ForwardsTracker right_motor_tracker(&right_motors,
                                    track_width / 2,
                                    wheel_diameter,
                                    final_rpm);

ForwardsTracker forwards_tracker(&forwards_odom_rotation, 0.1_in, 1.9654_in);
SidewaysTracker sideways_tracker(&sideways_odom_rotation, 1.0_in, 1.9869_in);

TrackingImu tracking_imu(&imu);

ArcOdomTracker arc_pose_tracker({ &forwards_tracker,
                                  &left_motor_tracker,
                                  &right_motor_tracker },
                                { &sideways_tracker },
                                { &tracking_imu });

// controller stuff
PID<Length, Voltage> linear_pid(4.5,
                                0.0,
                                3.6,
                                7,
                                // std::nullopt,
                                127,
                                50_msec,
                                1_in,
                                Voltage(1.0 / 127.0));

PID<Angle, Voltage>
  angular_pid(2.5, 0.0, 3.5, 14, 127, 50_msec, (1_stDeg), Voltage(1.0 / 127.0));

// tolerance stuff
Tolerances linearTolerances(200_msec,
                            ErrorTolerance { 3_in },
                            VelocityTolerance { 400_inps });
// HalfCircleTolerance { 1_in });

Tolerances angularTolerances(200_msec,
                             ErrorTolerance { 8_stDeg },
                             VelocityTolerance { 400_degps });

// large tolerances
Tolerances largeLinearTolerances(1_sec, ErrorTolerance { 5_in });
Tolerances largeAngularTolerances(1_sec, ErrorTolerance { 15_stDeg });

// chain tolerances
Tolerances chainLinearTolerances(1_sec, ErrorTolerance { 6_in });
Tolerances chainAngularTolerances(1_sec, ErrorTolerance { 20_stDeg });

normalLargeChainTolerances tolerances(linearTolerances,
                                      angularTolerances,
                                      largeLinearTolerances,
                                      largeAngularTolerances,

                                      chainLinearTolerances,
                                      chainAngularTolerances);

Chassis chassis(drivetrain, arc_pose_tracker, tolerances);

RunExecutor run;
AsyncExecutor async;

// MotionBuilder mb(chassis, controllers);
//
// ChainedExecutor chain(100_msec);

// characterizing linear:
// Linear:     1.1669 0.00595758   0.040725
//   vel = 1.1669 volt / mps
//   accel  = 0.00595758 volt / mps2
//   k_s  = 0.040725 volt
// characterizing angular:
// Angular:  -0.107902 -0.0107677 -0.0743809
//   vel = -0.107902 volt / radps
//   accel  = -0.0107677 volt / radps2
//   k_s  = -0.0743809 volt

blazing::lyfast::VelocityController velocity_controller(
  lyfast::VelocityControllerParams {
    .left_Kv = 0.426161 * volt / mps,
    .left_Ka = 0.0890043738963 * volt / mps2,
    .left_Ks = 0.0481902 * volt,
    .left_Kp = 0.934514846239 * volt / mps,
    .left_Ki = 4.58736473058 * volt / m,

    .right_Kv = 0.425642 * volt / mps,
    .right_Ka = 0.0915356456147 * volt / mps2,
    .right_Ks = 0.0499037 * volt,
    .right_Kp = 0.940127699096 * volt / mps,
    .right_Ki = 4.65515950473 * volt / m,
  },
  track_width);

Controllers controllers(
  // pid controllers
  PIDLinearController(linear_pid),
  PIDAngularController(angular_pid),
  lyfast::VelocityFeedforward<lyfast::VelocityController>(velocity_controller),

  // slew controllers
  LinearSlewController(0.07_volt, 0.06_volt),
  AngularSlewController(0.8_volt),

  // voltage constraints controllers
  // (included just so they can be set per motion)
  LinearVoltageClampController(),
  AngularVoltageClampController());

// void spline_test() {
//
//     blazing::lyfast::geometry::Line line({ -23.6_in, -23.6_in },
//                                          { -34.72_in, -39.79_in });
//     // blazing::lyfast::geometry::CubicBezier test_cubic({ -34.72_in,
//     -39.79_in
//     // },
//     //                                                   { -37.84_in,
//     -42.28_in
//     //                                                   }, { -43.38_in,
//     //                                                   -47.1_in }, {
//     -56_in,
//     //                                                   -47.1_in });
//
//     blazing::lyfast::geometry::CubicBezier test_cubic({ -34.72_in, -39.79_in
//     },
//                                                       { -36.58_in, -41.79_in
//                                                       }, { -36.86_in,
//                                                       -46.17_in }, { -56_in,
//                                                       -47.1_in });
//
//     blazing::lyfast::geometry::Spline spline({ &line, &test_cubic });
//
//     blazing::lyfast::mp::RobotConstraints robot_constraints(10.5_in,
//                                                             // 0.043,
//                                                             // 0.08,
//                                                             0.2,
//                                                             3.25_in,
//                                                             450_rpm,
//                                                             12_lb,
//                                                             6.0f);
//
//     blazing::lyfast::mp::LinearConstraints linear_constraints(70_inps,
//                                                               20.0_mps2,
//                                                               2.0_mps2);
//     // 1.6_mps2);
//     // effectively infinity
//     blazing::lyfast::mp::AngularConstraints angular_constraints(20_radps,
//                                                                 20_radps2,
//                                                                 20_radps2);
//
//     blazing::lyfast::mp::Constraints constraints(robot_constraints,
//                                                  linear_constraints,
//                                                  angular_constraints);
//
//     blazing::lyfast::mp::Trajectory cubic_trajectory(
//       &spline,
//       constraints,
//       {
//         // lyfast::mp::PointConstraint {
//         //                              .timeframe = 18_in,
//         //                              .vel = 10_inps,
//         //                              },
//       },
//       10_inps,
//       0_inps,
//       0.1_in);
//
//     // print out final trajectory and debug info
//
//     auto print =
//       []<typename T>(std::string name, std::vector<T>& list, T target_units)
//       {
//           std::cout << name << "=\\left[";
//           for (size_t i = 0; i < list.size(); i++) {
//               if (i != 0) std::cout << ",";
//               std::cout << list[i].convert(target_units);
//           }
//           std::cout << "\\right]" << std::endl;
//       };
//
//     print("a_{kin}", cubic_trajectory.max_kin_accel_debug, Finps2);
//     print("a_{turn}", cubic_trajectory.max_turn_accel_debug, Finps2);
//     print("d_{kin}", cubic_trajectory.max_kin_decel_debug, Finps2);
//     print("d_{turn}", cubic_trajectory.max_turn_decel_debug, Finps2);
//     //
//     print("v_{kin}", cubic_trajectory.max_kin_vel_debug, Finps);
//     print("v_{turn}", cubic_trajectory.max_turn_vel_debug, Finps);
//     print("v_{friction}", cubic_trajectory.max_friction_vel_debug, Finps);
//
//     print("v_{forward}", cubic_trajectory.forwards_pass_debug, Finps);
//     print("v_{backward}", cubic_trajectory.backwards_pass_debug, Finps);
//
//     print("v_{final}", cubic_trajectory.final_vels_debug, Finps);
//
//     std::cout << "l_{times}=\\left[";
//     for (auto& point : cubic_trajectory.points) {
//         std::cout << point.travel_time.convert(sec) << ",";
//     }
//     std::cout << "\\right]" << std::endl;
//
//     std::cout << "l_{points}=\\left[";
//     for (auto& point : cubic_trajectory.points) {
//         std::cout << "\\left(" << point.point.x.convert(in) << ","
//                   << point.point.y.convert(in) << "\\right),";
//     }
//     std::cout << "\\right]" << std::endl;
//
//     std::cout << "l_{headings}=\\left[";
//     for (auto& point : cubic_trajectory.points) {
//         std::cout << point.heading.internal() << ",";
//     }
//     std::cout << "\\right]" << std::endl;
//
//     drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
//
//     // run spline on ramsette
//     blazing::lyfast::Ramsete(controllers,
//                              chassis,
//                              &cubic_trajectory,
//                              0.7,
//                              35.0) |
//       run;
// }

void initialize() {
    // pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
    imu.reset(true);

    pros::Task([&] {
        while (true) {
            arc_pose_tracker.update();
            pros::delay(10);
        }
    });
}

// void manual_vel_testing() {
//     auto start_time = now();
//
//     while (true) {
//         // DifferentialSpeeds new_speeds = { 10_inps, 0_rps };
//         DifferentialSpeeds new_speeds = { 0_inps, 0.3_rps };
//         Time delta_time = 10_msec;
//
//         DifferentialVoltages voltages =
//           controllers.velocity_feedforward.update(new_speeds, delta_time);
//
//         std::array<Voltage, 2> saturated_voltages { voltages.left_voltage,
//                                                     voltages.right_voltage };
//
//         // normalizes voltages to [-1, 1]
//         auto [normal_left_voltage, normal_right_voltage] =
//           desaturate(saturated_voltages, 1_volt);
//
//         drivetrain.moveTank(normal_left_voltage, normal_right_voltage);
//
//         auto vel = arc_pose_tracker.getLinearVelocity();
//         auto ang_vel = arc_pose_tracker.getAngularVelocity();
//
//         // std::cout
//         //   << std::format(
//         //        "vel now is {:.2f}, unsat {:.2f},{:.2f}, " "satura "
//         //                                                   "{:.2f},{:.2f},
//         //                                                   pose "
//         //                                                   "{:.2f} {:.2f},
//         //                                                   time "
//         //                                                   "{:.2f}",
//         //        vel.convert(inps) / 2.0,
//         //        saturated_voltages[0].convert(volt),
//         //        saturated_voltages[1].convert(volt),
//         //        normal_left_voltage.convert(volt),
//         //        normal_right_voltage.convert(volt),
//         //        arc_pose_tracker.getPosition().x.convert(in),
//         //        arc_pose_tracker.getPosition().y.convert(in),
//         //        (now() - start_time).convert(sec))
//         //   << std::endl;
//
//         std::cout << std::format(
//                        "ang vel {:.4f}, unsat {:.2f},{:.2f}, satura "
//                        "{:.2f},{:.2f}, angle {:.3f}, time {:.2f}",
//                        ang_vel.convert(rps),
//                        saturated_voltages[0].convert(volt),
//                        saturated_voltages[1].convert(volt),
//                        normal_left_voltage.convert(volt),
//                        normal_right_voltage.convert(volt),
//                        arc_pose_tracker.getAngle().convert(deg),
//                        // arc_pose_tracker.getPosition().x.convert(in),
//                        // arc_pose_tracker.getPosition().y.convert(in),
//                        (now() - start_time).convert(sec))
//                   << std::endl;
//         pros::delay(15);
//     }
// }

void stanley_test() {

    blazing::lyfast::geometry::Line line({ -23.6_in, -23.6_in },
                                         { -34.72_in, -39.79_in });
    blazing::lyfast::geometry::CubicBezier test_cubic({ -34.72_in, -39.79_in },
                                                      { -36.58_in, -41.79_in },
                                                      { -36.86_in, -46.17_in },
                                                      { -56_in, -47.1_in });

    blazing::lyfast::geometry::Spline spline({ &line, &test_cubic });

    blazing::lyfast::mp::RobotConstraints robot_constraints(10.5_in,
                                                            // 0.043,
                                                            // 0.08,
                                                            0.3,
                                                            3.25_in,
                                                            450_rpm,
                                                            12_lb,
                                                            6.0f);

    blazing::lyfast::mp::LinearConstraints linear_constraints(40_inps,
                                                              20.0_mps2,
                                                              2.0_mps2);
    // 1.6_mps2);
    // effectively infinity
    blazing::lyfast::mp::AngularConstraints angular_constraints(20_radps,
                                                                20_radps2,
                                                                20_radps2);

    blazing::lyfast::mp::Constraints constraints(robot_constraints,
                                                 linear_constraints,
                                                 angular_constraints);

    blazing::lyfast::mp::Trajectory cubic_trajectory(
      &spline,
      constraints,
      {
        // lyfast::mp::PointConstraint {
        //                              .timeframe = 18_in,
        //                              .vel = 10_inps,
        //                              },
      },
      10_inps,
      0_inps,
      0.1_in);

    // print out final trajectory and debug info

    auto print =
      []<typename T>(std::string name, std::vector<T>& list, T target_units) {
          std::cout << name << "=\\left[";
          for (size_t i = 0; i < list.size(); i++) {
              if (i != 0) std::cout << ",";
              std::cout << list[i].convert(target_units);
          }
          std::cout << "\\right]" << std::endl;
      };

    bool printing = false;
    if (printing) {
        print("a_{kin}", cubic_trajectory.max_kin_accel_debug, Finps2);
        print("a_{turn}", cubic_trajectory.max_turn_accel_debug, Finps2);
        print("d_{kin}", cubic_trajectory.max_kin_decel_debug, Finps2);
        print("d_{turn}", cubic_trajectory.max_turn_decel_debug, Finps2);
        //
        print("v_{kin}", cubic_trajectory.max_kin_vel_debug, Finps);
        print("v_{turn}", cubic_trajectory.max_turn_vel_debug, Finps);
        print("v_{friction}", cubic_trajectory.max_friction_vel_debug, Finps);

        print("v_{forward}", cubic_trajectory.forwards_pass_debug, Finps);
        print("v_{backward}", cubic_trajectory.backwards_pass_debug, Finps);

        print("v_{final}", cubic_trajectory.final_vels_debug, Finps);

        std::cout << "l_{times}=\\left[";
        for (auto& point : cubic_trajectory.points) {
            std::cout << point.travel_time.convert(sec) << ",";
        }
        std::cout << "\\right]" << std::endl;

        std::cout << "l_{points}=\\left[";
        for (auto& point : cubic_trajectory.points) {
            std::cout << "\\left(" << point.point.x.convert(in) << ","
                      << point.point.y.convert(in) << "\\right),";
        }
        std::cout << "\\right]" << std::endl;

        std::cout << "l_{headings}=\\left[";
        for (auto& point : cubic_trajectory.points) {
            std::cout << point.heading.internal() << ",";
        }
        std::cout << "\\right]" << std::endl;
    }

    // drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);

    // run spline on ramsette
    // blazing::lyfast::Ramsete(controllers,
    //                          chassis,
    //                          &cubic_trajectory,
    //                          0.7,
    //                          35.0) |
    //   run;

    // run spline on stanley
    blazing::lyfast::Stanley(controllers, chassis, &cubic_trajectory)
        .k(0.5 / sec) |
      run;
}

void sysid() {
    std::vector<lyfast::SysIdVoltageCommands> left_voltage_commands;
    std::vector<lyfast::SysIdVoltageCommands> right_voltage_commands;

    // linear commands
    // for (int left_motor = 0; left_motor <= 1; left_motor++) {
    // for (int i = -10; i <= 10; i += 1) {
    //     voltage_commands.emplace_back(left_motor * i * 0.1 * volt,
    //                                   (1 - left_motor) * i * 0.1 * volt,
    //                                   1.0_sec);
    // }

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // left_voltage_commands.emplace_back(0_volt, 0_volt, 300_msec);
    std::vector<lyfast::SysIdVoltageCommands> mixed_voltage_commands = {
        // linear movements
        { 0.5_volt,  0.5_volt,  500_msec },
        { 0.7_volt,  0.7_volt,  600_msec },
        { 0.2_volt,  0.2_volt,  600_msec },
        { 0.0_volt,  0.0_volt,  300_msec },
        { -0.7_volt, -0.7_volt, 800_msec },
        { -0.2_volt, -0.2_volt, 800_msec },
        { 0.0_volt,  0.0_volt,  300_msec },

        { 0.5_volt,  0.5_volt,  500_msec },
        { 0.7_volt,  0.7_volt,  600_msec },
        { 0.2_volt,  0.2_volt,  600_msec },
        { 0.0_volt,  0.0_volt,  300_msec },
        { -0.7_volt, -0.7_volt, 800_msec },
        { -0.2_volt, -0.2_volt, 800_msec },
        { 0.0_volt,  0.0_volt,  300_msec },

        // angular
        { 0.5_volt,  -0.5_volt, 500_msec },
        { 1.0_volt,  -1.0_volt, 400_msec },
        { -0.5_volt, 0.5_volt,  800_msec },
        { -0.2_volt, 0.2_volt,  800_msec },
        { 1.0_volt,  -1.0_volt, 400_msec },
        { -0.2_volt, 0.2_volt,  300_msec },
        { 0.5_volt,  -0.5_volt, 500_msec },
        { 1.0_volt,  -1.0_volt, 400_msec },
        { -0.5_volt, 0.5_volt,  800_msec },
        { -0.2_volt, 0.2_volt,  800_msec },
        { 1.0_volt,  -1.0_volt, 400_msec },
        { -0.2_volt, 0.2_volt,  300_msec },
        { 0.5_volt,  -0.5_volt, 500_msec },
        { 1.0_volt,  -1.0_volt, 400_msec },
        { -0.5_volt, 0.5_volt,  800_msec },
        { -0.2_volt, 0.2_volt,  800_msec },
        { 1.0_volt,  -1.0_volt, 400_msec },
        { -0.2_volt, 0.2_volt,  300_msec },

        { -0.5_volt, 0.5_volt,  500_msec },
        { -1.0_volt, 1.0_volt,  400_msec },
        { 0.5_volt,  -0.5_volt, 800_msec },
        { 0.2_volt,  -0.2_volt, 800_msec },
        { -1.0_volt, 1.0_volt,  400_msec },
        { 0.2_volt,  -0.2_volt, 300_msec },
        { -0.5_volt, 0.5_volt,  500_msec },
        { -1.0_volt, 1.0_volt,  400_msec },
        { 0.5_volt,  -0.5_volt, 800_msec },
        { 0.2_volt,  -0.2_volt, 800_msec },
        { -1.0_volt, 1.0_volt,  400_msec },
        { 0.2_volt,  -0.2_volt, 300_msec },
        { -0.5_volt, 0.5_volt,  500_msec },
        { -1.0_volt, 1.0_volt,  400_msec },
        { 0.5_volt,  -0.5_volt, 800_msec },
        { 0.2_volt,  -0.2_volt, 800_msec },
        { -1.0_volt, 1.0_volt,  400_msec },
        { 0.2_volt,  -0.2_volt, 300_msec },
    };

    // for (int i = -10; i <= 10; i += 1) {
    //     if (i == 0)
    //         left_voltage_commands.emplace_back(0 * volt, 0 * volt, 0.3_sec);
    //     else
    //         left_voltage_commands.emplace_back(i * 0.1 * volt,
    //                                            0 * volt,
    //                                            0.5_sec);
    // }

    // for (int i = -10; i <= 10; i += 1) {
    //     if (i == 0)
    //         right_voltage_commands.emplace_back(0 * volt,
    //                                             i * 0 * volt,
    //                                             0.3_sec);
    //     else
    //         right_voltage_commands.emplace_back(0 * volt,
    //                                             i * 0.1 * volt,
    //                                             0.5_sec);
    // }

    auto mixed_linear_data = lyfast::createData(mixed_voltage_commands,
                                                &left_motors,
                                                &right_motors,
                                                3.25_in,
                                                450_rpm);

    // auto left_data = lyfast::createData(left_voltage_commands,
    //                                     &left_motors,
    //                                     &right_motors,
    //                                     3.25_in,
    //                                     450_rpm);

    drivetrain.setBrakeMode(pros::MotorBrake::brake);
    left_motors.move(0);
    right_motors.move(0);
    pros::delay(1000);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // auto right_data = lyfast::createData(right_voltage_commands,
    //                                      &left_motors,
    //                                      &right_motors,
    //                                      3.25_in,
    //                                      450_rpm);

    // voltage_commands.emplace_back(0 * volt, 0 * volt, 1.0_sec, false);
    // }

    // angular commands
    // for (int sign = -1; sign <= 1; sign += 2) {
    //     for (double i = 0.1; i <= 1; i += 0.1) {
    //         voltage_commands.emplace_back(i * -1 * sign * volt,
    //                                       i * sign * volt,
    //                                       1.0_sec);
    //     }
    //     voltage_commands.emplace_back(0 * volt, 0 * volt, 1.0_sec,
    //     false);
    // }

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "mixed linear data: " << std::endl;
            lyfast::printData(mixed_linear_data);

            // std::cout << "actual left motor data: " << std::endl;
            // lyfast::printData(left_data);
            // std::cout << "actual right motor data: " << std::endl;
            // lyfast::printData(right_data);
        }
        pros::delay(10);
    }
}

void sysid2() {
    // std::vector<lyfast::SysIdVoltageCommands> voltage_commands = {
    //     // linear movements
    //     { -0.1_volt, -0.1_volt, 400_msec  },
    //     { 0.2_volt,  0.2_volt,  1000_msec },
    //     { -0.3_volt, -0.3_volt, 1000_msec },
    //     { 0.4_volt,  0.4_volt,  1000_msec },
    //     { -0.5_volt, -0.5_volt, 1000_msec },
    //     { 0.6_volt,  0.6_volt,  1000_msec },
    //     { -0.7_volt, -0.7_volt, 1000_msec },
    // };
    //
    // auto data = lyfast::calculate_kv_ks(voltage_commands,
    //                                     &left_motors,
    //                                     &right_motors,
    //                                     3.25_in,
    //                                     450_rpm);

    std::vector<lyfast::SysIdVoltageCommands> mixed_voltage_commands = {
        // linear movements
        { 0.5_volt,  0.5_volt,  500_msec },
        { 0.7_volt,  0.7_volt,  600_msec },
        { 0.2_volt,  0.2_volt,  600_msec },
        { 0.0_volt,  0.0_volt,  300_msec },
        { -0.7_volt, -0.7_volt, 800_msec },
        { -0.2_volt, -0.2_volt, 800_msec },
        { 0.0_volt,  0.0_volt,  300_msec },

        { 0.5_volt,  0.5_volt,  500_msec },
        { 0.7_volt,  0.7_volt,  600_msec },
        { 0.2_volt,  0.2_volt,  600_msec },
        { 0.0_volt,  0.0_volt,  300_msec },
        { -0.7_volt, -0.7_volt, 800_msec },
        { -0.2_volt, -0.2_volt, 800_msec },
        { 0.0_volt,  0.0_volt,  300_msec },

        // angular
        { 0.5_volt,  -0.5_volt, 500_msec },
        { 1.0_volt,  -1.0_volt, 400_msec },
        { -0.5_volt, 0.5_volt,  800_msec },
        { -0.2_volt, 0.2_volt,  800_msec },
        { 1.0_volt,  -1.0_volt, 400_msec },
        { -0.2_volt, 0.2_volt,  300_msec },

        // { 0.5_volt,  -0.5_volt, 500_msec },
        // { 1.0_volt,  -1.0_volt, 400_msec },
        // { -0.5_volt, 0.5_volt,  800_msec },
        // { -0.2_volt, 0.2_volt,  800_msec },
        // { 1.0_volt,  -1.0_volt, 400_msec },
        // { -0.2_volt, 0.2_volt,  300_msec },
        //
        // { 0.5_volt,  -0.5_volt, 500_msec },
        // { 1.0_volt,  -1.0_volt, 400_msec },
        // { -0.5_volt, 0.5_volt,  800_msec },
        // { -0.2_volt, 0.2_volt,  800_msec },
        // { 1.0_volt,  -1.0_volt, 400_msec },
        // { -0.2_volt, 0.2_volt,  300_msec },

        // { -0.5_volt, 0.5_volt,  500_msec },
        // { -1.0_volt, 1.0_volt,  400_msec },
        // { 0.5_volt,  -0.5_volt, 800_msec },
        // { 0.2_volt,  -0.2_volt, 800_msec },
        // { -1.0_volt, 1.0_volt,  400_msec },
        // { 0.2_volt,  -0.2_volt, 300_msec },
        //
        // { -0.5_volt, 0.5_volt,  500_msec },
        // { -1.0_volt, 1.0_volt,  400_msec },
        // { 0.5_volt,  -0.5_volt, 800_msec },
        // { 0.2_volt,  -0.2_volt, 800_msec },
        // { -1.0_volt, 1.0_volt,  400_msec },
        // { 0.2_volt,  -0.2_volt, 300_msec },

        { -0.5_volt, 0.5_volt,  500_msec },
        { -1.0_volt, 1.0_volt,  400_msec },
        { 0.5_volt,  -0.5_volt, 800_msec },
        { 0.2_volt,  -0.2_volt, 800_msec },
        { -1.0_volt, 1.0_volt,  400_msec },
        { 0.2_volt,  -0.2_volt, 300_msec },
    };

    auto accel_data =
      lyfast::calculate_ka(mixed_voltage_commands,
                           &left_motors,
                           &right_motors,
                           3.25_in,
                           450_rpm,
                           velocity_controller.getParams().left_Kv,
                           velocity_controller.getParams().left_Ks,
                           velocity_controller.getParams().right_Kv,
                           velocity_controller.getParams().right_Ks);

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            // std::cout << "vel data: " << std::endl;
            // lyfast::printData(data);
            std::cout << "accel_data: " << std::endl;
            lyfast::printData(accel_data);
        }
        pros::delay(10);
    }
}

void sysid3() {
    std::vector<lyfast::SysIdVoltageCommands> accel_voltage_commands = {
        // linear movements
        { 0.5_volt, 0.5_volt, 2_sec },
    };

    auto accel_data = lyfast::createData(accel_voltage_commands,
                                         &left_motors,
                                         &right_motors,
                                         3.25_in,
                                         450_rpm);

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            // std::cout << "vel data: " << std::endl;
            // lyfast::printData(data);
            std::cout << "accel_data: " << std::endl;
            lyfast::printData(accel_data);
        }
        pros::delay(10);
    }
}

void opcontrol() {
    // sysid3();
    // sysid2();

    std::vector<std::pair<DifferentialSpeeds, lyfast::LeftRightSpeeds>> data;
    std::vector<LeftRightVoltages> voltages;

    Time start_time = from_msec(pros::millis());

    LinearAcceleration max_acceleration = 50_inps2;
    LinearVelocity max_velocity = 40_inps;

    Length distance = 54_in;

    Time end_time = 2 * distance / units::sqrt(distance * max_acceleration);

    auto t_4 = (max_velocity / max_acceleration);
    auto t_5 = (max_velocity / max_acceleration +
                (distance -
                 (1.f / 4.f * max_velocity / max_acceleration * max_velocity)) /
                  max_velocity);
    auto b = (distance - units::pow<2>(max_velocity) / max_acceleration);

    while (true) {
        Time curr_time = from_msec(pros::millis());
        Time motion_time = curr_time - start_time;

        if (motion_time >= end_time) {
            break;
        }

        // double throttle =
        // master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); double turn
        // = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // //
        // throttle /= 127.0;
        // turn /= 127.0;
        //
        // // constexpr auto max_vel = (7.5_rpm * 3.25_in * M_PI / rad);
        // constexpr auto max_vel = 2_mps;
        // // constexpr auto max_rad = 2_mps * ;
        // //
        // DifferentialSpeeds target {
        //     .linear_velocity = throttle * max_vel,
        //     .angular_velocity = turn * rad * (max_vel / (track_width *
        //     0.5))
        // };

        auto mp = [&](Time time) -> LinearVelocity {
            if (b < 0.0_in) {
                if (time < end_time / 2) {
                    return (max_acceleration * time);
                } else {
                    return (-max_acceleration * (time - end_time / 2.f) +
                            max_acceleration * end_time / 2.f);
                }
            } else {
                if (time < t_4) {
                    return (max_acceleration * time);
                } else {
                    if (time <= t_5) {
                        return (max_velocity);
                    } else {
                        return (-max_acceleration * (time - t_5) +
                                max_velocity);
                    }
                }
            }
            // targets velocity 10 msec into the future
        };

        // target speed in the future
        LinearVelocity curr_target_speed = mp(motion_time + 10_msec);

        // use desired speed now in logs
        LinearVelocity desired_curr_speed = mp(motion_time);

        DifferentialSpeeds target { .linear_velocity = curr_target_speed,
                                    .angular_velocity = 0_radps };

        DifferentialSpeeds desired_target { .linear_velocity =
                                              desired_curr_speed,
                                            .angular_velocity = 0_radps };

        LinearVelocity curr_left_vel =
          LinearVelocity((left_motors.get_actual_velocity(0) +
                          left_motors.get_actual_velocity(1) +
                          left_motors.get_actual_velocity(2)) *
                         0.00324173091942 / 3.0);
        LinearVelocity curr_right_vel =
          LinearVelocity((right_motors.get_actual_velocity(0) +
                          right_motors.get_actual_velocity(1) +
                          right_motors.get_actual_velocity(2)) *
                         0.00324173091942 / 3.0);

        LinearVelocity curr_lin_vel = (curr_left_vel + curr_right_vel) / 2.0;

        LeftRightVoltages volts =
          // controllers.velocity_feedforward.update(target, 10_msec);
          controllers.velocity_feedforward.update(
            { curr_left_vel, curr_right_vel },
            target,
            10_msec);
        //
        //
        // double leftPower = throttle + turn;
        // double rightPower = throttle - turn;

        left_motors.move_voltage(to_mvolt(12 * volts.left_voltage));
        right_motors.move_voltage(to_mvolt(12 * volts.right_voltage));

        // std::cout << "volt l/r: " << volts.left_voltage.internal() << " "
        //           << volts.right_voltage.internal() << std::endl;

        // std::cout << "targetv l/a: " << target.linear_velocity.internal()
        // << " "
        //           << target.angular_velocity.internal() << std::endl;

        // std::cout << "l/r: "
        //           << (left_motors.get_actual_velocity(0) +
        //               left_motors.get_actual_velocity(1) +
        //               left_motors.get_actual_velocity(2)) *
        //                0.00324173091942 / 3.0
        //           << " "
        //           << (right_motors.get_actual_velocity(0) +
        //               right_motors.get_actual_velocity(1) +
        //               right_motors.get_actual_velocity(2)) *
        //                0.00324173091942 / 3.0
        //           << std::endl;

        data.emplace_back(
          desired_target,
          lyfast::LeftRightSpeeds { curr_left_vel, curr_right_vel });
        voltages.emplace_back(volts);

        pros::delay(10);
    }

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            // std::cout << "vel data: " << std::endl;
            // lyfast::printData(data);
            std::cout << "data: " << std::endl;
            std::cout << "\\left[";
            for (auto [target, actual] : data) {
                std::cout << "\\left(" << target.linear_velocity.internal()
                          << ","
                          << (actual.left_vel + actual.right_vel).internal() /
                               2.0
                          << "\\right),";
            }
            std::cout << "\\right]," << std::endl;

            std::cout << "voltages:";
            std::cout << "\\left[";
            for (auto curr_voltages : voltages) {
                std::cout << "\\left(" << curr_voltages.left_voltage.internal()
                          << "," << curr_voltages.right_voltage.internal()
                          << "\\right),";
            }

            std::cout << "\\right]," << std::endl;
        }
        pros::delay(10);
    }

    // arc_pose_tracker.setPose({ -23.6_in, -23.6_in, 270_stDeg });
    // std::cout << "what!" << std::endl;
    // stanley_test();
    // std::cout << "finished motion!" << std::endl;
    // drivetrain.moveTank(0_volt, 0_volt);
}
