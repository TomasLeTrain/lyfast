#include "main.h"
#include "blazing/api.hpp"
#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/utils.hpp"
#include "lyfast/api.hpp"
#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/motor_group.hpp"
#include "pros/optical.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
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


int8_t left_front = 15;
int8_t left_middle = -14;
int8_t left_back = -12;

int8_t right_front = -17;
int8_t right_middle = 16;
int8_t right_back = 19;

bool vexmaps_logging_enabled = true;
bool custom_particling = true;

pros::MotorGroup left_motors({ left_front, left_middle, left_back }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::MotorGroup right_motors({ right_front, right_middle, right_back }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
// clang-format on

ScaledIMU imu(20, (360.0 + 1.5) / 360.0);

pros::Controller master(pros::E_CONTROLLER_MASTER);

// odom rotation sensors
// pros::Rotation forwards_odom_rotation(-20);
pros::Rotation forwards_odom_rotation(-13);
pros::Rotation sideways_odom_rotation(18);

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

// tracker configs - same signs as lemlib
// tracker_config_t forwards_tracker_config = {
//     .diameter = 1.991_in,
//     // geometric is also 0
//     .offset = -0.08_in,
// };
//
// tracker_config_t sideways_tracker_config = {
//     .diameter = 1.991_in,
//     // geometric are -2.5, meaning cor is 0.5_in forwards from geometric
//     center .offset = -3.17_in,
// };

// TODO: update since now sideways might be zero
ForwardsTracker forwards_tracker(&forwards_odom_rotation, 0.5_in, 1.991_in);
SidewaysTracker sideways_tracker(&sideways_odom_rotation, -2.6_in, 1.991_in);

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
                                127, // max
                                std::nullopt, // derivative alpha
                                50_msec,
                                1_in,
                                Voltage(1.0 / 127.0));

PID<Angle, Voltage> angular_pid(2.5,
                                0.0,
                                3.5,
                                14,
                                127, // max
                                std::nullopt, // derivative alpha
                                50_msec,
                                (1_stDeg),
                                Voltage(1.0 / 127.0));

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

blazing::lyfast::DifferentialVelocityController linear_velocity_controller(
  lyfast::VelocityControllerParams {
    .left_Kv = 0.42 * volt / mps,

    // length kp and ka term create a feedback loop intenuating noise
    .left_Ka = 0.08 * volt / mps2,
    .left_Ks = 0.08 * volt,

    .left_Kp = 0.7 * volt / mps,
    .left_Ki = 4.0 * volt / m,

    .right_Kv = 0.43 * volt / mps,
    .right_Ka = 0.09 * volt / mps2,
    .right_Ks = 0.08 * volt,

    .right_Kp = 0.7 * volt / mps,
    .right_Ki = 4.0 * volt / m,
  },
  70_inps,
  track_width,
  0.8,
  false, // prioritize angular everywhere?
  std::ref(drivetrain));

// --- turning vel stuff --- //
// goated for turning
blazing::lyfast::DifferentialVelocityController angular_velocity_controller(
  lyfast::VelocityControllerParams {

    .left_Kv = 0.47 * volt / mps,
    .left_Ka = 0.09 * volt / mps2,
    .left_Ks = 0.08 * volt,

    .left_Kp = 0.3 * volt / mps,
    .left_Ki = 5.09538143189 * volt / m,

    .right_Kv = 0.475 * volt / mps,
    .right_Ka = 0.09 * volt / mps2,
    .right_Ks = 0.08 * volt,

    .right_Kp = 0.3 * volt / mps,
    .right_Ki = 5.5578634857 * volt / m,
  },
  76_inps,
  track_width,
  0.85,
  false, // TODO: shouldn't affect swings?
  std::ref(drivetrain));

lyfast::ArcadeVelocityController vel_controller {
    linear_velocity_controller,
    angular_velocity_controller,
    76_inps,
    false, // prioritize angular everywhere?
    track_width
};

PID<Length, LinearVelocity> linear_vel_pid(0.5,
                                           0.0,
                                           3.6,
                                           7,
                                           // std::nullopt,
                                           127, // max
                                           std::nullopt, // derivative alpha
                                           50_msec,
                                           1_in,
                                           1_inps);

CascadedControllers<decltype(linear_vel_pid),
                    decltype(vel_controller),
                    Length,
                    LinearVelocity,
                    Voltage>
  linear_control(linear_vel_pid, vel_controller);

PID<Angle, AngularVelocity> angular_vel_pid(4.5,
                                            0.0,
                                            3.6,
                                            7,
                                            // std::nullopt,
                                            127, // max
                                            std::nullopt, // derivative alpha
                                            50_msec,
                                            1_stDeg,
                                            1_degps);

CascadedControllers<decltype(angular_vel_pid),
                    decltype(vel_controller),
                    Angle,
                    AngularVelocity,
                    Voltage>
  angular_control(angular_vel_pid, vel_controller);

FLength track_radius = track_width * 0.5;

FLinearVelocity max_velocity = 76_Finps;
// w = v / r
FAngularVelocity max_angular_velocity = (max_velocity / track_radius) * Frad;

std::array<float, 3> Q { // max forwards of 10 inches?
                         (20_in).internal(),
                         // max crosstrack of 6 inches?
                         (10_in).internal(),
                         // maximum is 180
                         (90_stDeg).internal()
};
std::array<float, 2> R { // max velocity
                         max_velocity.internal(),
                         // max angular velocity
                         max_angular_velocity.internal()
};

blazing::lyfast::state_space::LTVUnicycleController lqr_controller(Q, R);

lyfast::PathPoseFeedbackController<decltype(lqr_controller)>
  path_pose_feedback_controller(lqr_controller);

Controllers controllers(
  // pid controllers
  // PIDLinearController(linear_pid),
  // PIDAngularController(angular_pid),

  LinearFeedbackController<decltype(linear_control)>(linear_control),
  AngularFeedbackController<decltype(angular_control)>(angular_control),

  lyfast::VelocityFeedforward<lyfast::ArcadeVelocityController>(vel_controller),

  path_pose_feedback_controller,

  // slew controllers
  // LinearSlewController(0.07_volt, 0.06_volt),
  // AngularSlewController(0.8_volt),

  LinearSlewController {},
  AngularSlewController {},

  // voltage constraints controllers
  // (included just so they can be set per motion)
  LinearVoltageClampController(),
  AngularVoltageClampController());

MotionBuilder mb(chassis, controllers);
MotionBuilder vel_mb(chassis, controllers);

ChainedExecutor chain(100_msec);

void initialize() {
    // pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
    imu.reset(true);

    pros::Task([&] {
        while (true) {
            arc_pose_tracker.update();
            pros::delay(10);
        }
    });

    // default a timeout
    // vel_mb.setTurnToModifier([](auto&& turnTo) {
    //     return std::move(
    //       turnTo
    //         .velocity_based(true)
    //         // specifically uses turn heading pid instead of drive pid
    //         .timeout(5_sec));
    // });
    //
    // vel_mb.setDistanceAtHeadingModifier([](auto&& distanceAtHeading) {
    //     return
    //     std::move(distanceAtHeading.velocity_based(true).timeout(5_sec));
    // });
    //
    // vel_mb.setMoveToModifier([](auto&& moveTo) {
    //     // return moveTo.customAngularLinearFunc(angular_linear_func);
    //     return std::move(
    //       moveTo.velocity_based(true).k_lat(0.15 * rad / m).timeout(3_sec));
    //     // return moveTo.timeout(3_sec);
    //     // .customAngularLinearFunc(angular_linear_func);
    // });
    //
    // vel_mb.setBoomerangModifier([](auto&& boomerang) {
    //     // return boomerang.customAngularLinearFunc(angular_linear_func);
    //     // return boomerang.k_lat();
    //     return std::move(boomerang.velocity_based(true)
    //                        .k_lat(0.15 * rad / m, true)
    //                        .timeout(5_sec));
    //     // .customAngularLinearFunc(angular_linear_func);
    // });
}

// void stanley_test() {
//     using namespace blazing::lyfast;
//     using namespace blazing::lyfast::geometry;
//     using namespace blazing::lyfast::mp;
//
//     Line line({ -23.6_in, -23.6_in }, { -34.72_in, -39.79_in });
//     CubicBezier test_cubic({ -34.72_in, -39.79_in },
//                            { -36.58_in, -41.79_in },
//                            { -36.86_in, -46.17_in },
//                            { -56_in, -47.1_in });
//
//     Spline spline({ &line, &test_cubic });
//
//     RobotConstraints robot_constraints(10.5_in,
//                                        // 0.043,
//                                        // 0.08,
//                                        0.3,
//                                        3.25_in,
//                                        450_rpm,
//                                        12_lb,
//                                        6.0f);
//
//     LinearConstraints linear_constraints(40_inps, 20.0_mps2, 2.0_mps2);
//     // 1.6_mps2);
//     // effectively infinity
//     AngularConstraints angular_constraints(20_radps, 20_radps2, 20_radps2);
//
//     Constraints constraints(robot_constraints,
//                             linear_constraints,
//                             angular_constraints);
//
//     Trajectory cubic_trajectory(
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
//     bool printing = false;
//     if (printing) {
//         print("a_{kin}", cubic_trajectory.max_kin_accel_debug, Finps2);
//         print("a_{turn}", cubic_trajectory.max_turn_accel_debug, Finps2);
//         print("d_{kin}", cubic_trajectory.max_kin_decel_debug, Finps2);
//         print("d_{turn}", cubic_trajectory.max_turn_decel_debug, Finps2);
//         //
//         print("v_{kin}", cubic_trajectory.max_kin_vel_debug, Finps);
//         print("v_{turn}", cubic_trajectory.max_turn_vel_debug, Finps);
//         print("v_{friction}", cubic_trajectory.max_friction_vel_debug,
//         Finps);
//
//         print("v_{forward}", cubic_trajectory.forwards_pass_debug, Finps);
//         print("v_{backward}", cubic_trajectory.backwards_pass_debug, Finps);
//
//         print("v_{final}", cubic_trajectory.final_vels_debug, Finps);
//
//         std::cout << "l_{times}=\\left[";
//         for (auto& point : cubic_trajectory.points) {
//             std::cout << point.travel_time.convert(sec) << ",";
//         }
//         std::cout << "\\right]" << std::endl;
//
//         std::cout << "l_{points}=\\left[";
//         for (auto& point : cubic_trajectory.points) {
//             std::cout << "\\left(" << point.point.x.convert(in) << ","
//                       << point.point.y.convert(in) << "\\right),";
//         }
//         std::cout << "\\right]" << std::endl;
//
//         std::cout << "l_{headings}=\\left[";
//         for (auto& point : cubic_trajectory.points) {
//             std::cout << point.heading.internal() << ",";
//         }
//         std::cout << "\\right]" << std::endl;
//     }
//
//     // drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
//
//     // run spline on ramsette
//     // blazing::lyfast::Ramsete(controllers,
//     //                          chassis,
//     //                          &cubic_trajectory,
//     //                          0.7,
//     //                          35.0) |
//     //   run;
//
//     // run spline on stanley
//     Stanley(controllers, chassis, &cubic_trajectory).k(0.5 / sec) | run;
// }

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void kv_ks_tuner(
  std::string type,
  std::vector<lyfast::DifferentialSysIdVoltageCommands> voltage_commands,
  Time delta_time = 10_msec) {
    lyfast::DifferentialSysidData data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {

            std::cout << "type: " << type << std::endl;

            data = lyfast::DifferentialSysid::calculate_kv_ks(voltage_commands,
                                                              drivetrain);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "kv/ks type: " << type << std::endl;
            lyfast::DifferentialSysid::printData(data, delta_time);
        }
        pros::delay(10);
    }
}

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void raw_ka_tuner(
  std::string type,
  std::vector<lyfast::DifferentialSysIdVoltageCommands> voltage_commands,
  lyfast::KvUnits left_Kv,
  lyfast::KsUnits left_Ks,
  lyfast::KvUnits right_Kv,
  lyfast::KsUnits right_Ks) {
    lyfast::DifferentialSysidData data;

    Time delta_time = 10_msec;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            data = lyfast::DifferentialSysid::calculate_ka(voltage_commands,
                                                           drivetrain,
                                                           left_Kv,
                                                           left_Ks,
                                                           right_Kv,
                                                           right_Ks,
                                                           delta_time);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "data" << std::endl;
            lyfast::DifferentialSysid::printData(data, delta_time);
        }
        pros::delay(10);
    }
}

void create_accel_data(lyfast::DifferentialSysIdVoltageCommands voltage_command,
                       std::string type) {
    Time delta_time = 10_msec;

    std::vector<lyfast::DifferentialSysIdVoltageCommands>
      accel_voltage_commands = { // linear movements
                                 // { u_step, u_step, 2_sec },
                                 voltage_command
      };

    lyfast::DifferentialSysidData accel_data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            accel_data =
              lyfast::DifferentialSysid::createData(accel_voltage_commands,
                                                    drivetrain,
                                                    delta_time);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "accel_data: " << std::endl;
            std::cout << "u_step (l,r): " << voltage_command.left_voltage
                      << ", " << voltage_command.right_voltage << std::endl;
            lyfast::DifferentialSysid::printData(accel_data, delta_time);
        }
        pros::delay(10);
    }
}

void ka_kp_ki_tuner(std::string type,
                    lyfast::DifferentialSysIdVoltageCommands voltage_command,
                    double lambda_factor,
                    Time delta_time = 10_msec) {
    lyfast::DifferentialSysidData data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            data = lyfast::DifferentialSysid::calculate_ka_kp_ki_fopdt(
              voltage_command,
              drivetrain,
              delta_time,
              lambda_factor);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "data: " << std::endl;
            lyfast::DifferentialSysid::printData(data, delta_time);
        }
        pros::delay(10);
    }
}

void linear_ka_kp_ki_tuner(Voltage u_step = 0.5_volt,
                           double lambda_factor = 0.6,
                           Time accel_time = 2_sec,
                           Time delta_time = 10_msec) {

    ka_kp_ki_tuner("LINEAR",
                   { u_step, u_step, accel_time },
                   lambda_factor,
                   delta_time);
}

void angular_ka_kp_ki_tuner(Voltage u_step = 0.5_volt,
                            double lambda_factor = 0.6,
                            Time accel_time = 2_sec,
                            Time delta_time = 10_msec) {
    ka_kp_ki_tuner("ANGULAR",
                   { u_step, -u_step, accel_time },
                   lambda_factor,
                   delta_time);
}

void linear_kv_ks_tuner(Time delta_time = 10_msec) {
    kv_ks_tuner("LINEAR",
                std::vector<lyfast::DifferentialSysIdVoltageCommands> {
                  // linear movements
                  { -0.1_volt, -0.1_volt, 600_msec  },
                  { 0.2_volt,  0.2_volt,  1000_msec },
                  { -0.3_volt, -0.3_volt, 1000_msec },
                  { 0.4_volt,  0.4_volt,  1000_msec },
                  { -0.5_volt, -0.5_volt, 1000_msec },
                  { 0.6_volt,  0.6_volt,  1000_msec },
                  { -0.7_volt, -0.7_volt, 1000_msec },
    },
                delta_time);
}

void angular_kv_ks_tuner(Time delta_time = 10_msec) {
    kv_ks_tuner("ANGULAR",
                std::vector<lyfast::DifferentialSysIdVoltageCommands> {
                  // linear movements
                  { 0.1_volt,  -0.1_volt, 600_msec  },
                  { -0.2_volt, 0.2_volt,  1000_msec },
                  { 0.3_volt,  -0.3_volt, 1000_msec },
                  { -0.4_volt, 0.4_volt,  1000_msec },
                  { 0.5_volt,  -0.5_volt, 1000_msec },
                  { -0.6_volt, 0.6_volt,  1000_msec },
                  { 0.7_volt,  -0.7_volt, 1000_msec },
    },
                delta_time);
}

void linear_raw_ka_tuner(lyfast::KvUnits left_Kv,
                         lyfast::KsUnits left_Ks,
                         lyfast::KvUnits right_Kv,
                         lyfast::KsUnits right_Ks) {
    std::vector<lyfast::DifferentialSysIdVoltageCommands>
      mixed_voltage_commands = {
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
    };
    raw_ka_tuner("LINEAR",
                 mixed_voltage_commands,
                 left_Kv,
                 left_Ks,
                 right_Kv,
                 right_Ks);
}

void angular_raw_ka_tuner(lyfast::KvUnits left_Kv,
                          lyfast::KsUnits left_Ks,
                          lyfast::KvUnits right_Kv,
                          lyfast::KsUnits right_Ks) {
    std::vector<lyfast::DifferentialSysIdVoltageCommands>
      mixed_voltage_commands = {
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
    raw_ka_tuner("LINEAR",
                 mixed_voltage_commands,
                 left_Kv,
                 left_Ks,
                 right_Kv,
                 right_Ks);
}

void manual_mp_test() {
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
            // assume velocity of 0 everywhere outside mp
            if (time > end_time || time < 0_sec) return 0_mps;

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
                          << actual.right_vel.internal() << "\\right),";
            }
            std::cout << "\\right]" << std::endl;

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
}

void simple_mp_test() {
    std::vector<std::pair<LeftRightSpeeds, LeftRightSpeeds>> data;
    std::vector<LeftRightVoltages> voltages;

    LinearVelocity max_vel = 60_inps;
    LinearAcceleration max_accel = 150_inps2;
    LinearAcceleration max_decel = 60_inps2;
    Length target = 48_in;

    motions::simple_mp::TrapezoidalProfileTrajectory trajectory(max_vel,
                                                                max_accel,
                                                                max_decel,
                                                                target);

    Time start_time = from_msec(pros::millis());

    while (true) {
        Time curr_time = from_msec(pros::millis());
        Time motion_time = curr_time - start_time;

        if (motion_time >= trajectory.total_time) {
            break;
        }

        // target speed in the future
        LinearVelocity curr_target_speed =
          trajectory.getVelocity(motion_time + 10_msec);

        // use desired speed now in logs
        LinearVelocity desired_curr_speed = trajectory.getVelocity(motion_time);

        DifferentialSpeeds target { .linear_velocity = curr_target_speed,
                                    .angular_velocity = 0_radps };

        DifferentialSpeeds desired_target { .linear_velocity =
                                              desired_curr_speed,
                                            .angular_velocity = 0_radps };

        LinearVelocity curr_left_vel =
          drivetrain.getDrivetrainVelocities().left_vel;
        LinearVelocity curr_right_vel =
          drivetrain.getDrivetrainVelocities().right_vel;

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
                          << actual.right_vel.internal() << "\\right),";
            }
            std::cout << "\\right]" << std::endl;

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
}

void path_follow_test() {
    using namespace blazing::lyfast;
    using namespace blazing::lyfast::geometry;
    using namespace blazing::lyfast::mp;
    arc_pose_tracker.setPose({ 0_in, 0_in, 0_stDeg });

    std::shared_ptr<Line> line(new Line({ -23.6_in, 0_in }, { 0_in, 0_in }));
    std::shared_ptr<CubicBezier> bezier(new CubicBezier({ 0_in, 0_in },
                                                        { 23.6_in, 0_in },
                                                        { 23.6_in, 23.6_in },
                                                        { 47.2_in, 23.6_in }));

    // TODO: fix whatever is wrong with spline thingy
    std::shared_ptr<geometry::Spline> spline_ptr { new Spline(
      { line, bezier }) };
    //   { bezier }) };

    RobotConstraints robot_constraints(
      10.5_in, // track with
      0.9, // friction coeff
      3.25_in, // wheel diameter
      410_rpm, // max ang vel - determined somewhat from data
      14.8_lb, // about 6.7 kg
      2.0f); // motor count - determined somewhat from data

    LinearConstraints linear_constraints(
      60_inps, // max vel - for testing
      // 20.0_inps2, // max accel - for testing
      10000.0_inps2, // max accel - for testing
      100_inps2 // max decel - for testing also
    );
    // 1.6_mps2);
    // effectively infinity
    AngularConstraints angular_constraints(2.0_radps, 1.3_radps2, 1.3_radps2);

    Constraints constraints(robot_constraints,
                            linear_constraints,
                            angular_constraints);

    // std::make_shared<geometry::Curve>(spline);

    std::shared_ptr<Trajectory> test_trajectory(
      new Trajectory(bezier,
                     constraints,
                     {
                       // lyfast::mp::PointConstraint {
                       //                              .timeframe = 18_in,
                       //                              .vel = 10_inps,
                       //                              },
                     },
                     // some initial velocity for it to move?
                     // TODO: could there be a place on the curve that also has
                     // a velof zero? if so this would also have the same issue?
                     1_inps,
                     0_inps,
                     0.1_in));

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

    bool printing = true;
    if (printing) {
        print("a_{kin}", test_trajectory->max_kin_accel_debug, Finps2);
        print("a_{turn}", test_trajectory->max_turn_accel_debug, Finps2);
        print("d_{kin}", test_trajectory->max_kin_decel_debug, Finps2);
        print("d_{turn}", test_trajectory->max_turn_decel_debug, Finps2);
        //
        print("v_{kin}", test_trajectory->max_kin_vel_debug, Finps);
        print("v_{turn}", test_trajectory->max_turn_vel_debug, Finps);
        print("v_{friction}", test_trajectory->max_friction_vel_debug, Finps);

        print("v_{forward}", test_trajectory->forwards_pass_debug, Finps);
        print("v_{backward}", test_trajectory->backwards_pass_debug, Finps);

        print("v_{final}", test_trajectory->final_vels_debug, Finps);

        std::cout << "l_{times}=\\left[";
        for (auto& point : test_trajectory->points) {
            std::cout << point.travel_time.convert(sec) << ",";
        }
        std::cout << "\\right]" << std::endl;

        std::cout << "l_{points}=\\left[";
        for (auto& point : test_trajectory->points) {
            std::cout << "\\left(" << point.point.x.convert(in) << ","
                      << point.point.y.convert(in) << "\\right),";
        }
        std::cout << "\\right]" << std::endl;

        std::cout << "l_{headings}=\\left[";
        for (auto& point : test_trajectory->points) {
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

    std::cout << "running path!" << std::endl;
    // use path follow to follow the path
    lyfast::PathFollow<decltype(controllers),
                       decltype(drivetrain),
                       decltype(arc_pose_tracker),
                       decltype(tolerances)>(controllers,
                                             chassis,
                                             test_trajectory)
        .drive_toleranceDuration(100_sec)
        .drive_largeToleranceDuration(100_sec)
        .timeout(5_sec) |
      run;
}

void opcontrol() {
    // TODO: how to actually tune these?
    //   lqr_controller.setQMatrix(Q);
    //   lqr_controller.setRMatrix(R);
    //
    //   // assume robot is at 0,0
    //   lyfast::PathPoseFeedbackT state {
    //       .pose = { 0_in, 0_in, 0_stDeg },
    //       .velocities = {.linear_velocity = 0_inps, .angular_velocity =
    //       0_radps,}
    //   };
    //
    //   // want to reach (10, 10) with angle of 10 degrees and with some
    //   // velocity
    //   lyfast::PathPoseFeedbackT reference {
    //       .pose = { 1_in, 1_in, 10_stDeg },
    //       .velocities = { .linear_velocity = 10_inps, .angular_velocity =
    //       1_radps,
    // }
    //   };
    //
    //   lqr_controller.setState(
    //     lyfast::state_space::LTVUnicycleController::State::fromPathPoseFeedback(
    //       (state)));
    //
    //   lqr_controller.setReference(
    //     lyfast::state_space::LTVUnicycleController::State::fromPathPoseFeedback(
    //       (reference)));
    //
    //   auto start_time = pros::micros();
    //   lqr_controller.compute();
    //   auto end_time = pros::micros();
    //
    //   std::cout << "LQR in " << end_time - start_time << " micro seconds"
    //             << std::endl;
    //
    //   auto result = lqr_controller.getInput();
    //   // auto result = lqr_controller.update(state, reference, 10_msec);
    //
    //   if (result.has_value()) {
    //       std::cout << "lqr returned: " <<
    //       result->linear_velocity.convert(inps)
    //                 << " inps " << result->angular_velocity << std::endl;
    //   } else {
    //       std::cout << "LQR encountered an error" << std::endl;
    //   }

    pros::delay(2000);
    path_follow_test();

    // pros::delay(2000);
    // linear_ka_kp_ki_tuner();
    // // create_accel_data({ 0.5_volt, 0.5_volt, 2_sec }, "Linear");
    // // //
    // // return;
    //
    // manual_mp_test();
    // stanley_test();

    // Length distance = 10_in;
    // LinearVelocity start_vel = 0_inps;
    // LinearVelocity end_vel = 0_inps;
    //
    // lyfast::geometry::Point start { 0_in, 0_in };
    // lyfast::geometry::Point end { distance, 0_in };
    //
    // lyfast::geometry::Line line(start, end);
    //
    // Length track_width = 10.5_in;
    // float coeff_friction = 0.9;
    // Length wheel_diameter = 3.25_in;
    // AngularVelocity final_rpm = 450_rpm;
    // Mass robot_mass = 14.8_lb;
    // float motor_count = 6.0;
    //
    // LinearVelocity max_vel = 70_inps;
    // LinearAcceleration max_accel = 10000_inps2;
    // LinearAcceleration max_decel = 10000_inps2;
    //
    // lyfast::mp::RobotConstraints robot_constraints(track_width,
    //                                                coeff_friction,
    //                                                wheel_diameter,
    //                                                final_rpm,
    //                                                robot_mass,
    //                                                motor_count);
    // lyfast::mp::LinearConstraints linear_constraints(max_vel,
    //                                                  max_accel,
    //                                                  max_decel);
    // lyfast::mp::AngularConstraints angular_constraints {
    //     AngularVelocity(20_radps),
    //     FAngularAcceleration(200_radps2),
    //     AngularAcceleration(200_radps2)
    // };
    //
    // lyfast::mp::Constraints constraints(robot_constraints,
    //                                     linear_constraints,
    //                                     angular_constraints);
    //
    // lyfast::mp::Trajectory trajectory(&line,
    //                                   constraints,
    //                                   {},
    //                                   start_vel,
    //                                   end_vel,
    //                                   0.1_in);
    //
    // auto print =
    //   []<typename T>(std::string name, std::vector<T>& list, T
    //   target_units)
    //   {
    //       std::cout << name << "=\\left[";
    //       for (size_t i = 0; i < list.size(); i++) {
    //           if (i != 0) std::cout << ",";
    //           std::cout << list[i].convert(target_units);
    //       }
    //       std::cout << "\\right]" << std::endl;
    //   };
    //
    // bool printing = true;
    // if (printing) {
    //     print("a_{kin}", trajectory.max_kin_accel_debug, Finps2);
    //     print("a_{turn}", trajectory.max_turn_accel_debug, Finps2);
    //     print("d_{kin}", trajectory.max_kin_decel_debug, Finps2);
    //     print("d_{turn}", trajectory.max_turn_decel_debug, Finps2);
    //     //
    //     print("v_{kin}", trajectory.max_kin_vel_debug, Finps);
    //     print("v_{turn}", trajectory.max_turn_vel_debug, Finps);
    //     print("v_{friction}", trajectory.max_friction_vel_debug, Finps);
    //
    //     print("v_{forward}", trajectory.forwards_pass_debug, Finps);
    //     print("v_{backward}", trajectory.backwards_pass_debug, Finps);
    //
    //     print("v_{final}", trajectory.final_vels_debug, Finps);
    //
    //     std::cout << "l_{times}=\\left[";
    //     for (auto& point : trajectory.points) {
    //         std::cout << point.travel_time.convert(sec) << ",";
    //     }
    //     std::cout << "\\right]" << std::endl;
    //
    //     std::cout << "l_{points}=\\left[";
    //     for (auto& point : trajectory.points) {
    //         std::cout << "\\left(" << point.point.x.convert(in) << ","
    //                   << point.point.y.convert(in) << "\\right),";
    //     }
    //     std::cout << "\\right]" << std::endl;
    //
    //     std::cout << "l_{headings}=\\left[";
    //     for (auto& point : trajectory.points) {
    //         std::cout << point.heading.internal() << ",";
    //     }
    //     std::cout << "\\right]" << std::endl;
    // }
    //
    // // mb.moveTo(20, 20).velocity_based(true) | run;
    // // mb.turnTo(20, 20).velocity_based(true) | run;
    // // mb.boomerang(20, 20, 0).velocity_based(true) | run;
    // // mb.arc(20, 20, 1_in).velocity_based(true) | run;
    //
    // // arc_pose_tracker.setPose({ -23.6_in, -23.6_in, 270_stDeg });
    // // std::cout << "what!" << std::endl;
    // // stanley_test();
    // // std::cout << "finished motion!" << std::endl;
    // // drivetrain.moveTank(0_volt, 0_volt);
}
