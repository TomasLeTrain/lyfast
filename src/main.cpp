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

Length track_width = 10.5_in;
Length wheel_diameter = 3.25_in;
AngularVelocity final_rpm = 450_rpm;

// tracker stuff
DifferentialDrivetrain
  drivetrain(&left_motors, &right_motors, wheel_diameter, final_rpm);

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

blazing::lyfast::VelocityController linear_velocity_controller(
  lyfast::VelocityControllerParams {

    // auto tuner constants
    // .left_Kv = 0.424124 * volt / mps,
    // .left_Ka = 0.102243 * volt / mps2,
    // .left_Ks = 0.0598427 * volt,
    // .left_Kp = 0.946491 * volt / mps,
    // .left_Ki = 4.38097 * volt / m,
    //
    // .right_Kv = 0.418423 * volt / mps,
    // .right_Ka = 0.110488 * volt / mps2,
    // .right_Ks = 0.0638434 * volt,
    // .right_Kp = 0.943091 * volt / mps,
    // .right_Ki = 4.02498 * volt / m,

    // desmos constants
    // .left_Kv = 0.426161 * volt / mps,
    // .left_Ka = 0.0890043738963 * volt / mps2,
    // .left_Ks = 0.0481902 * volt,
    // .left_Kp = 0.934514846239 * volt / mps,
    // .left_Ki = 4.58736473058 * volt / m,
    //
    // .right_Kv = 0.425642 * volt / mps,
    // .right_Ka = 0.0915356456147 * volt / mps2,
    // .right_Ks = 0.0499037 * volt,
    // .right_Kp = 0.940127699096 * volt / mps,
    // .right_Ki = 4.65515950473 * volt / m,

    // tuned with one left motor unplugged
    // .left_Kv = 0.678023 * volt / mps,
    // .left_Ka = 0.220584 * volt / mps2,
    // .left_Ks = 0.0665045 * volt,
    // .left_Kp = 1.41625 * volt / mps,
    // .left_Ki = 4.54646 * volt / m,
    //
    // .right_Kv = 0.42346 * volt / mps,
    // .right_Ka = 0.11003 * volt / mps2,
    // .right_Ks = 0.0699037 * volt,
    // .right_Kp = 0.947525 * volt / mps,
    // .right_Ki = 4.07981 * volt / m,

    // custom accel
    // .left_Kv = 0.426161 * volt / mps,
    // .left_Ka = 0.08 * volt / mps2,
    // .left_Ks = 0.0481902 * volt,
    // .left_Kp = 0.934514846239 * volt / mps,
    // .left_Ki = 4.58736473058 * volt / m,
    //
    // .right_Kv = 0.425642 * volt / mps,
    // .right_Ka = 0.081 * volt / mps2,
    // .right_Ks = 0.0499037 * volt,
    // .right_Kp = 0.940127699096 * volt / mps,
    // .right_Ki = 4.65515950473 * volt / m,

		// auto tuner acceleration is not really good atm
		// using desmos constants for now
    .left_Kv = 0.427641833333 * volt / mps,
    .left_Ka = 0.0918263592271 * volt / mps2,
    .left_Ks = 0.0546282666667 * volt,
		// lambda 0.55
    .left_Kp = 0.857349891834 * volt / mps,
    .left_Ki = 4.40262320937 * volt / m,

    .right_Kv = 0.424849333333 * volt / mps,
    .right_Ka = 0. * volt / mps2,
    .right_Ks = 0.05694315 * volt,
    .right_Kp = 0. * volt / mps,
    .right_Ki = 4 * volt / m,
  },
  track_width,
  drivetrain);

blazing::lyfast::VelocityController angular_velocity_controller(
  lyfast::VelocityControllerParams {
    // desmos constants
    .left_Kv = 0.451918 * volt / mps,
    .left_Ka = 0.1457828 * volt / mps2,
    .left_Ks = 0.0922515 * volt,
    .left_Kp = 0.9984206 * volt / mps,
    .left_Ki = 3.457622 * volt / m,

    .right_Kv = 0.477938 * volt / mps,
    .right_Ka = 0.132789 * volt / mps2,
    .right_Ks = 0.0824404 * volt,
    .right_Kp = 1.054508 * volt / mps,
    .right_Ki = 4.24337 * volt / m,
  },
  track_width,
  drivetrain);

Controllers controllers(
  // pid controllers
  PIDLinearController(linear_pid),
  PIDAngularController(angular_pid),
  lyfast::VelocityFeedforward<lyfast::VelocityController>(
    linear_velocity_controller),

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

void get_linear_kv_ks() {
    std::vector<lyfast::SysIdVoltageCommands> voltage_commands = {
        // linear movements
        { -0.1_volt, -0.1_volt, 400_msec  },
        { 0.2_volt,  0.2_volt,  1000_msec },
        { -0.3_volt, -0.3_volt, 1000_msec },
        { 0.4_volt,  0.4_volt,  1000_msec },
        { -0.5_volt, -0.5_volt, 1000_msec },
        { 0.6_volt,  0.6_volt,  1000_msec },
        { -0.7_volt, -0.7_volt, 1000_msec },
    };

    std::cout << "LINEAR DATA" << std::endl;
    auto data = lyfast::calculate_kv_ks(voltage_commands, drivetrain);

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "kv/ks data: " << std::endl;
            lyfast::printData(data);
        }
        pros::delay(10);
    }
}

void get_angular_kv_ks() {
    std::vector<lyfast::SysIdVoltageCommands> voltage_commands = {
        // linear movements
        { 0.1_volt,  -0.1_volt, 400_msec  },
        { -0.2_volt, 0.2_volt,  1000_msec },
        { 0.3_volt,  -0.3_volt, 1000_msec },
        { -0.4_volt, 0.4_volt,  1000_msec },
        { 0.5_volt,  -0.5_volt, 1000_msec },
        { -0.6_volt, 0.6_volt,  1000_msec },
        { 0.7_volt,  -0.7_volt, 1000_msec },
    };

    std::cout << "ANGULAR DATA" << std::endl;
    auto data = lyfast::calculate_kv_ks(voltage_commands, drivetrain);

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "kv/ks data: " << std::endl;
            lyfast::printData(data);
        }
        pros::delay(10);
    }
}

void linear_squares_accel() {
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
                           linear_velocity_controller.getParams().left_Kv,
                           linear_velocity_controller.getParams().left_Ks,
                           linear_velocity_controller.getParams().right_Kv,
                           linear_velocity_controller.getParams().right_Ks);

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "accel_data: " << std::endl;
            lyfast::printData(accel_data);
        }
        pros::delay(10);
    }
}

void create_fopdt_data() {
    Voltage u_step = 0.5_volt;

    std::vector<lyfast::SysIdVoltageCommands> accel_voltage_commands = {
        // linear movements
        { u_step, u_step, 2_sec },
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
            std::cout << "accel_data: " << std::endl;
            std::cout << "u_step of " << u_step << std::endl;
            lyfast::printData(accel_data);
        }
        pros::delay(10);
    }
}

void linear_ka_kp_ki_sysid() {
    Voltage u_step = 0.5_volt;
    std::cout << "LINEAR DATA" << std::endl;

    double lambda = 0.6;
    auto data = lyfast::calculate_ka_kp_ki_fopdt({ u_step, u_step, 2_sec },
                                                 drivetrain,
                                                 lambda);

    drivetrain.moveVoltages({ 0_volt, 0_volt });

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "data: " << std::endl;
            lyfast::printData(data);
        }
        pros::delay(10);
    }
}

void angular_ka_kp_ki_sysid() {
    Voltage u_step = 0.5_volt;
    std::cout << "ANGULAR DATA" << std::endl;
    lyfast::calculate_ka_kp_ki_fopdt({ u_step, -u_step, 2_sec }, drivetrain);

    drivetrain.moveVoltages({ 0_volt, 0_volt });
}

void opcontrol() {
    pros::delay(2000);
    // get_linear_kv_ks();
    linear_ka_kp_ki_sysid();

    return;

    std::vector<std::pair<LeftRightSpeeds, LeftRightSpeeds>> data;
    std::vector<LeftRightVoltages> voltages;

    Time start_time = from_msec(pros::millis());

    LinearAcceleration max_acceleration = 150_inps2;
    LinearVelocity max_velocity = 60_inps;
    Length distance = 48_in;

    // time to reach max velocity
    Time accel_time = (max_velocity / max_acceleration);

    Length accel_dist = 0.5 * max_acceleration * accel_time * accel_time;
    Length steady_dist = distance - 2 * accel_dist;

    Time steady_time = steady_dist / max_velocity;

    // decel_start_time = total time - time to decelerate to zero
    Time decel_start_time = accel_time + steady_time;

    Time end_time = steady_time + 2 * accel_time;

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
            if (steady_dist < 0.0_in) {
                if (time < end_time / 2) {
                    return (max_acceleration * time);
                } else {
                    return (-max_acceleration * (time - end_time * 0.5) +
                            // max vel we got to
                            max_acceleration * (end_time * 0.5));
                }
            } else {
                if (time < accel_time) {
                    return (max_acceleration * time);
                } else {
                    if (time <= decel_start_time) {
                        return (max_velocity);
                    } else {
                        return (-max_acceleration * (time - decel_start_time) +
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

        // #0. test autos overshooting way too much, if its localization thing
        // #2. make two separate controllers, one linear and one angular ???
        // #2.5. just make separate controllers with left/right
        // #3. make kv / ks tuner work for both configurations
        // #4. make ka tuner with FOPDT
        // #4. add kp/ki calcuations (free)

        LinearVelocity curr_left_vel =
          drivetrain.getDrivetrainVelocities().left_vel;
        LinearVelocity curr_right_vel =
          drivetrain.getDrivetrainVelocities().right_vel;

        LinearVelocity curr_lin_vel = (curr_left_vel + curr_right_vel) / 2.0;

        LeftRightVoltages volts =
          controllers.velocity_feedforward.update(target, 10_msec);
        // controllers.velocity_feedforward.update(
        //   { curr_left_vel, curr_right_vel },
        //   target,
        //   10_msec);

        drivetrain.moveTank(volts.left_voltage, volts.right_voltage);

        data.emplace_back(
          LeftRightSpeeds { desired_curr_speed, desired_curr_speed },
          LeftRightSpeeds { curr_left_vel, curr_right_vel });
        voltages.emplace_back(volts);

        pros::delay(10);
    }

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            // std::cout << "vel data: " << std::endl;
            // lyfast::printData(data);
            std::cout << "LEFT MOTORS: " << std::endl;
            std::cout << "\\left[";
            for (auto [target, actual] : data) {
                std::cout << "\\left(" << target.left_vel.internal() << ","
                          << actual.left_vel.internal() << "\\right),";
            }
            std::cout << "\\right]" << std::endl;

            std::cout << "RIGHT MOTORS: " << std::endl;
            std::cout << "\\left[";
            for (auto [target, actual] : data) {
                std::cout << "\\left(" << target.right_vel.internal() << ","
                          << actual.right_vel.internal() << "\\right)";
            }
            std::cout << "\\right]," << std::endl;

            std::cout << "VOLTAGES:" << std::endl;
            std::cout << "\\left[";
            for (auto curr_voltages : voltages) {
                std::cout << "\\left(" << curr_voltages.left_voltage.internal()
                          << "," << curr_voltages.right_voltage.internal()
                          << "\\right),";
            }

            std::cout << "\\right]" << std::endl;
        }
        pros::delay(10);
    }

    // arc_pose_tracker.setPose({ -23.6_in, -23.6_in, 270_stDeg });
    // std::cout << "what!" << std::endl;
    // stanley_test();
    // std::cout << "finished motion!" << std::endl;
    // drivetrain.moveTank(0_volt, 0_volt);
}
