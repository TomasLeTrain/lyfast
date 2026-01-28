#include "main.h"
#include "blazing/api.hpp"
#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/utils.hpp"
#include "lyfast/api.hpp"
#include "lyfast/motion_profiling/simple_mp.hpp"
#include "lyfast/system_identification.hpp"
#include "lyfast/vel_controller.hpp"
#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/motor_group.hpp"
#include "pros/optical.h"
#include "units/Angle.hpp"
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

lyfast::LinearAngularVelocityController vel_controller {
    linear_velocity_controller,
    angular_velocity_controller
};

PID<Length, LinearVelocity> linear_vel_pid(0.5,
                                           0.0,
                                           3.6,
                                           7,
                                           // std::nullopt,
                                           127,
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
                                            127,
                                            50_msec,
                                            1_stDeg,
                                            1_degps);

CascadedControllers<decltype(angular_vel_pid),
                    decltype(vel_controller),
                    Angle,
                    AngularVelocity,
                    Voltage>
  angular_control(angular_vel_pid, vel_controller);

Controllers controllers(
  // pid controllers
  // PIDLinearController(linear_pid),
  // PIDAngularController(angular_pid),

  LinearFeedbackController<decltype(linear_control)>(linear_control),
  AngularFeedbackController<decltype(angular_control)>(angular_control),

  lyfast::VelocityFeedforward<lyfast::LinearAngularVelocityController>(
    vel_controller),

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
}

void stanley_test() {
    using namespace blazing::lyfast;
    using namespace blazing::lyfast::geometry;
    using namespace blazing::lyfast::mp;

    Line line({ -23.6_in, -23.6_in }, { -34.72_in, -39.79_in });
    CubicBezier test_cubic({ -34.72_in, -39.79_in },
                           { -36.58_in, -41.79_in },
                           { -36.86_in, -46.17_in },
                           { -56_in, -47.1_in });

    Spline spline({ &line, &test_cubic });

    RobotConstraints robot_constraints(10.5_in,
                                       // 0.043,
                                       // 0.08,
                                       0.3,
                                       3.25_in,
                                       450_rpm,
                                       12_lb,
                                       6.0f);

    LinearConstraints linear_constraints(40_inps, 20.0_mps2, 2.0_mps2);
    // 1.6_mps2);
    // effectively infinity
    AngularConstraints angular_constraints(20_radps, 20_radps2, 20_radps2);

    Constraints constraints(robot_constraints,
                            linear_constraints,
                            angular_constraints);

    Trajectory cubic_trajectory(
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
    Stanley(controllers, chassis, &cubic_trajectory).k(0.5 / sec) | run;
}

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
}

void opcontrol() {
    // pros::delay(2000);
    linear_ka_kp_ki_tuner();
    create_accel_data({ 0.5_volt, 0.5_volt, 2_sec }, "Linear");
    //
    return;

    manual_mp_test();

    // arc_pose_tracker.setPose({ -23.6_in, -23.6_in, 270_stDeg });
    // std::cout << "what!" << std::endl;
    // stanley_test();
    // std::cout << "finished motion!" << std::endl;
    // drivetrain.moveTank(0_volt, 0_volt);
}
