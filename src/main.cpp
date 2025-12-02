#include "main.h"
#include "blazing/api.hpp"
#include "lyfast/api.hpp"
#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/motor_group.hpp"
#include "units/Vector2D.hpp"
#include <iostream>
#include <mutex>

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
blazing::lyfast::VelocityController
  velocity_controller((0.58345 - 0.05) * volt / mps,
                      0.00297879 * volt / mps2,
                      0.107902 * volt / radps,
                      0.0107677 * volt / radps2,
                      0.0041 * volt);

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

    blazing::lyfast::geometry::Line line({ -23.6_in, -23.6_in },
                                         { -34.72_in, -39.79_in });
    // blazing::lyfast::geometry::CubicBezier test_cubic({ -34.72_in, -39.79_in
    // },
    //                                                   { -37.84_in, -42.28_in
    //                                                   }, { -43.38_in,
    //                                                   -47.1_in }, { -56_in,
    //                                                   -47.1_in });

    blazing::lyfast::geometry::CubicBezier test_cubic({ -34.72_in, -39.79_in },
                                                      { -36.58_in, -41.79_in },
                                                      { -36.86_in, -46.17_in },
                                                      { -56_in, -47.1_in });

    blazing::lyfast::geometry::Spline spline({ &line, &test_cubic });

    blazing::lyfast::mp::RobotConstraints robot_constraints(10.5_in,
                                                            // 0.043,
                                                            // 0.08,
                                                            0.2,
                                                            3.25_in,
                                                            450_rpm,
                                                            12_lb,
                                                            6.0f);

    blazing::lyfast::mp::LinearConstraints linear_constraints(70_inps,
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

    drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);

    // run spline on ramsette
    blazing::lyfast::Ramsete(controllers,
                             chassis,
                             &cubic_trajectory,
                             0.7,
                             35.0) |
      run;

    // run spline on stanley
    // blazing::lyfast::Stanley(controllers, chassis, &spline_trajectory) | run;
}

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

void manual_vel_testing() {
    auto start_time = now();

    while (true) {
        // DifferentialSpeeds new_speeds = { 10_inps, 0_rps };
        DifferentialSpeeds new_speeds = { 0_inps, 0.3_rps };
        Time delta_time = 10_msec;

        DifferentialVoltages voltages =
          controllers.velocity_feedforward.update(new_speeds, delta_time);

        std::array<Voltage, 2> saturated_voltages { voltages.left_voltage,
                                                    voltages.right_voltage };

        // normalizes voltages to [-1, 1]
        auto [normal_left_voltage, normal_right_voltage] =
          desaturate(saturated_voltages, 1_volt);

        drivetrain.moveTank(normal_left_voltage, normal_right_voltage);

        auto vel = arc_pose_tracker.getLinearVelocity();
        auto ang_vel = arc_pose_tracker.getAngularVelocity();

        // std::cout
        //   << std::format(
        //        "vel now is {:.2f}, unsat {:.2f},{:.2f}, " "satura "
        //                                                   "{:.2f},{:.2f},
        //                                                   pose "
        //                                                   "{:.2f} {:.2f},
        //                                                   time "
        //                                                   "{:.2f}",
        //        vel.convert(inps) / 2.0,
        //        saturated_voltages[0].convert(volt),
        //        saturated_voltages[1].convert(volt),
        //        normal_left_voltage.convert(volt),
        //        normal_right_voltage.convert(volt),
        //        arc_pose_tracker.getPosition().x.convert(in),
        //        arc_pose_tracker.getPosition().y.convert(in),
        //        (now() - start_time).convert(sec))
        //   << std::endl;

        std::cout << std::format(
                       "ang vel {:.4f}, unsat {:.2f},{:.2f}, satura "
                       "{:.2f},{:.2f}, angle {:.3f}, time {:.2f}",
                       ang_vel.convert(rps),
                       saturated_voltages[0].convert(volt),
                       saturated_voltages[1].convert(volt),
                       normal_left_voltage.convert(volt),
                       normal_right_voltage.convert(volt),
                       arc_pose_tracker.getAngle().convert(deg),
                       // arc_pose_tracker.getPosition().x.convert(in),
                       // arc_pose_tracker.getPosition().y.convert(in),
                       (now() - start_time).convert(sec))
                  << std::endl;
        pros::delay(15);
    }
}

void opcontrol() {
    arc_pose_tracker.setPose({ -23.6_in, -23.6_in, 270_stDeg });
    spline_test();
    std::cout << "finished motion!" << std::endl;
    // drivetrain.moveTank(0_volt, 0_volt);
}
