#include "main.h"
#include "blazing/api.hpp"
#include "blazing/drivetrains/differential.hpp"
#include "lyfast/geometry/cubicBezier.hpp"
#include "lyfast/geometry/curve.hpp"
#include "lyfast/geometry/line.hpp"
#include "lyfast/geometry/spline.hpp"
#include "lyfast/motion_profiling/constraints.hpp"
#include "lyfast/motion_profiling/mp.hpp"
#include "lyfast/ramsete.hpp"
#include "lyfast/stanley.hpp"
#include "lyfast/vel_controller.hpp"
#include "pros/apix.h"
#include "units/Vector2D.hpp"
#include <iostream>
#include <mutex>

void initialize() {
    // pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
}

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

  private:
    const double m_scalar;
    int m_port;

    mutable pros::Mutex m_mutex;

    double m_offset = 0;
};

// clang-format off
// motor groups
pros::MotorGroup left_motors({ -12, -13, 14 }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::MotorGroup right_motors({ 7, 17, -16 }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
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

blazing::lyfast::VelocityController velocity_controller(0.5 * volt / mps,
                                                        0 * volt / mps2,
                                                        1 * volt / rps,
                                                        0.3 * volt / rps2,
                                                        0_volt);

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

void spline_test() {
    blazing::lyfast::geometry::CubicBezier first_cubic({ 15.35_in, 47.2_in },
                                                       { 23.2_in, 47.2_in },
                                                       { 23.2_in, 35_in },
                                                       { 15.1_in, 35_in });
    blazing::lyfast::geometry::Line line({ 15.1_in, 35_in }, { -40_in, 35_in });
    blazing::lyfast::geometry::CubicBezier second_cubic({ -40_in, 35_in },
                                                        { -50_in, 35_in },
                                                        { -44_in, 47.2_in },
                                                        { -56_in, 47.2_in });

    blazing::lyfast::geometry::Spline spline(
      { &first_cubic, &line, &second_cubic });
    // lyfast::geometry::Spline spline({ &first_cubic, &line, &second_cubic });

    blazing::lyfast::mp::RobotConstraints robot_constraints(10.5_in,
                                                            0.1,
                                                            3.25_in,
                                                            450_rpm,
                                                            13_lb,
                                                            6.0f);

    blazing::lyfast::mp::LinearConstraints linear_constraints(70_inps,
                                                              8.513_mps2,
                                                              8.513_mps2);
    // effectively infinity
    blazing::lyfast::mp::AngularConstraints angular_constraints(10_rps,
                                                                10_rps2,
                                                                10_rps2);

    blazing::lyfast::mp::Constraints constraints(robot_constraints,
                                                 linear_constraints,
                                                 angular_constraints);

    blazing::lyfast::mp::Trajectory spline_trajectory(&spline,
                                                      constraints,
                                                      0_mps,
                                                      0_mps,
                                                      0.1_in);

    // print out final trajectory and debug info

    // auto print =
    //   []<typename T>(std::string name, std::vector<T>& list, T target_units)
    //   {
    //       std::cout << name << "=\\left[";
    //       for (size_t i = 0; i < list.size(); i++) {
    //           if (i != 0) std::cout << ",";
    //           std::cout << list[i].convert(target_units);
    //       }
    //       std::cout << "\\right]" << std::endl;
    //   };

    // print("a_{kin}", spline_trajectory.max_kin_accel_debug, Finps2);
    // print("a_{turn}", spline_trajectory.max_turn_accel_debug, Finps2);
    // print("d_{kin}", spline_trajectory.max_kin_decel_debug, Finps2);
    // print("d_{turn}", spline_trajectory.max_turn_decel_debug, Finps2);
    // //
    // print("v_{kin}", spline_trajectory.max_kin_vel_debug, Finps);
    // print("v_{turn}", spline_trajectory.max_turn_vel_debug, Finps);
    // print("v_{friction}", spline_trajectory.max_friction_vel_debug, Finps);
    //
    // print("v_{forward}", spline_trajectory.forwards_pass_debug, Finps);
    // print("v_{backward}", spline_trajectory.backwards_pass_debug, Finps);
    //
    // print("v_{final}", spline_trajectory.final_vels_debug, Finps);
    //
    // std::cout << "l_{times}=\\left[";
    // for (auto& point : spline_trajectory.points) {
    //     std::cout << point.travel_time.convert(sec) << ",";
    // }
    // std::cout << "\\right]" << std::endl;
    //
    // std::cout << "l_{points}=\\left[";
    // for (auto& point : spline_trajectory.points) {
    //     std::cout << "\\left(" << point.point.x.convert(in) << ","
    //               << point.point.y.convert(in) << "\\right),";
    // }
    // std::cout << "\\right]" << std::endl;
    //
    // std::cout << "l_{headings}=\\left[";
    // for (auto& point : spline_trajectory.points) {
    //     std::cout << point.heading.internal() << ",";
    // }
    // std::cout << "\\right]" << std::endl;

    // run spline on ramsette
    blazing::lyfast::Ramsete(controllers,
                             chassis,
                             &spline_trajectory,
                             0.5,
                             0.5) |
      run;

    // run spline on stanley
    // blazing::lyfast::Stanley(controllers, chassis, &spline_trajectory) | run;
}

// void cubic_test() {
//     std::cout << "hello world!" << std::endl;
//
//     blazing::lyfast::geometry::CubicBezier cubic({ 2_in, 2_in },
//                                                  { 5_in, 15_in },
//                                                  { 20_in, 14_in },
//                                                  { 15_in, 5_in });
//
//     blazing::lyfast::mp::RobotConstraints robot_constraints(10.5_in,
//                                                             0.1,
//                                                             3.25_in,
//                                                             450_rpm,
//                                                             13_lb,
//                                                             6.0f);
//     blazing::lyfast::mp::LinearConstraints linear_constraints(90_inps,
//                                                               60_inps2,
//                                                               60_inps2);
//     blazing::lyfast::mp::AngularConstraints angular_constraints(0.8_rps,
//                                                                 1_rps2,
//                                                                 1_rps2);
//
//     blazing::lyfast::mp::Constraints constraints(robot_constraints,
//                                                  linear_constraints,
//                                                  angular_constraints);
//
//     blazing::lyfast::mp::Trajectory cubic_trajectory(&cubic,
//                                                      constraints,
//                                                      0_mps,
//                                                      0_mps,
//                                                      0.1_in);
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
//
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
//         std::cout << point.travel_time.internal() << ",";
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
// }

void opcontrol() {
    spline_test();
}
