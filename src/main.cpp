#include "main.h"
#include "blazing/api.hpp"
#include "blazing/utils.hpp"
#include "lyfast/api.hpp"
#include "lyfast/drivetrains/velocity_differential.hpp"
#include "lyfast/plants/velocity_plants.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/motor_group.hpp"
#include "pros/optical.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <cstdint>
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

lyfast::DifferentialVelocityControllerParams params {
	.linear = {
		.left_Kv = 0.43 * volt / mps,
		.left_Ka = 0.09 * volt / mps2,
		.left_Ks = 0.04 * volt,

		.right_Kv = 0.43 * volt / mps,
		.right_Ka = 0.09 * volt / mps2,
		.right_Ks = 0.04 * volt,
	},
	.angular = {
		.left_Kv = 0.47 * volt / mps,
		.left_Ka = 0.09 * volt / mps2,
		.left_Ks = 0.06 * volt,

		.right_Kv = 0.47 * volt / mps,
		.right_Ka = 0.09 * volt / mps2,
		.right_Ks = 0.06 * volt,
	},
	.pid = {
		.left_Kp = 0.7 * volt / mps,
		.left_Ki = 0.0 * volt / m,

		.left_max_output =  1_volt,
		.left_tbh_factor =  1.0,

		.right_Kp = 0.7 * volt / mps,
		.right_Ki = 0.0 * volt / m,

		.right_max_output =  1_volt,
		.right_tbh_factor =  1.0,
	}
};
lyfast::DifferentialVelocityController vel_controller { params,
                                                        76_inps,
                                                        track_width,
                                                        false };

lyfast::EMAVelocityFilter::Constants drivetrain_ema_filter_constants {
    .final_gearing_rpm = 450_rpm,
    .Koffset = 0.1,
    .KalphaFactor = 0.7,
};

lyfast::EMAVelocityFilter left_ema_filter { &left_motors,
                                            drivetrain_ema_filter_constants };
lyfast::EMAVelocityFilter right_ema_filter { &right_motors,
                                             drivetrain_ema_filter_constants };

lyfast::DrivetrainVelocityPlant drivetrain_plant { &left_ema_filter,
                                                   &right_ema_filter,
                                                   vel_controller,
                                                   wheel_diameter };

lyfast::VelocityDifferentialDrivetrain velocity_drivetrain(&left_motors,
                                                           &right_motors,
                                                           &drivetrain_plant,
                                                           track_width);

ForwardsTracker
  left_motor_tracker(&left_motors, -track_width / 2, wheel_diameter, final_rpm);

ForwardsTracker right_motor_tracker(&right_motors,
                                    track_width / 2,
                                    wheel_diameter,
                                    final_rpm);

// TODO: update since now sideways might be zero
ForwardsTracker forwards_tracker(&forwards_odom_rotation, 0.5_in, 1.991_in);
SidewaysTracker sideways_tracker(&sideways_odom_rotation, -2.6_in, 1.991_in);

TrackingImu tracking_imu(&imu);

ArcOdomTracker arc_pose_tracker({ &forwards_tracker,
                                  &left_motor_tracker,
                                  &right_motor_tracker },
                                // { &sideways_tracker },
                                {},
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
Chassis velocity_chassis(velocity_drivetrain, arc_pose_tracker, tolerances);

RunExecutor run;
AsyncExecutor async;

FLength track_radius = track_width * 0.5;

FLinearVelocity max_velocity = 76_Finps;
// w = v / r
FAngularVelocity max_angular_velocity = (max_velocity / track_radius) * Frad;

std::array<float, 3> Q { (40_in).internal(),
                         (3_in).internal(),
                         (45_stDeg).internal() };

std::array<float, 2> R { // max velocity
                         max_velocity.internal(),
                         // max angular velocity
                         max_angular_velocity.internal()
};

blazing::lyfast::state_space::LTVUnicycleController lqr_controller(Q, R);

lyfast::PathPoseFeedbackController<decltype(lqr_controller)>
  path_pose_feedback_controller(lqr_controller);

// blazing::lyfast::NoPathFeedbackController no_feedback_controller;
// lyfast::PathPoseFeedbackController<decltype(no_feedback_controller)>
// path_pose_feedback_controller(no_feedback_controller);

Controllers controllers(
  // pid controllers
  PIDLinearController(linear_pid),
  PIDAngularController(angular_pid),

  path_pose_feedback_controller,

  LinearSlewController {},
  AngularSlewController {},

  // voltage constraints controllers
  LinearVoltageClampController(),
  AngularVoltageClampController());

// MotionBuilder mb(chassis, controllers);
MotionBuilder vel_mb(velocity_chassis, controllers);

ChainedExecutor chain(100_msec);

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void kv_ks_tuner(
  std::string type,
  std::vector<lyfast::sysid::DifferentialVoltageCommand> voltage_commands,
  Time delta_time = 10_msec) {
    lyfast::sysid::DifferentialData data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {

            std::cout << "type: " << type << std::endl;

            data = lyfast::sysid::DifferentialUtils::calculate_kv_ks(
              voltage_commands,
              drivetrain);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "kv/ks type: " << type << std::endl;
            lyfast::sysid::DifferentialUtils::printData(data, delta_time);
        }
        pros::delay(10);
    }
}

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void raw_ka_tuner(
  std::string type,
  std::vector<lyfast::sysid::DifferentialVoltageCommand> voltage_commands,
  lyfast::KvUnits<LinearVelocity> left_Kv,
  lyfast::KsUnits left_Ks,
  lyfast::KvUnits<LinearVelocity> right_Kv,
  lyfast::KsUnits right_Ks) {
    lyfast::sysid::DifferentialData data;

    Time delta_time = 10_msec;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            data =
              lyfast::sysid::DifferentialUtils::calculate_ka(voltage_commands,
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
            lyfast::sysid::DifferentialUtils::printData(data, delta_time);
        }
        pros::delay(10);
    }
}

void create_accel_data(
  lyfast::sysid::DifferentialVoltageCommand voltage_command,
  std::string type) {
    Time delta_time = 10_msec;

    std::vector<lyfast::sysid::DifferentialVoltageCommand>
      accel_voltage_commands = { // linear movements
                                 // { u_step, u_step, 2_sec },
                                 voltage_command
      };

    lyfast::sysid::DifferentialData accel_data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            accel_data = lyfast::sysid::DifferentialUtils::createData(
              accel_voltage_commands,
              drivetrain,
              delta_time);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "accel_data: " << std::endl;
            std::cout << "u_step (l,r): " << voltage_command.left_voltage
                      << ", " << voltage_command.right_voltage << std::endl;
            lyfast::sysid::DifferentialUtils::printData(accel_data, delta_time);
        }
        pros::delay(10);
    }
}

void ka_kp_ki_tuner(std::string type,
                    lyfast::sysid::DifferentialVoltageCommand voltage_command,
                    double lambda_factor,
                    Time delta_time = 10_msec) {
    lyfast::sysid::DifferentialData data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            data = lyfast::sysid::DifferentialUtils::calculate_ka_kp_ki_fopdt(
              voltage_command,
              drivetrain,
              delta_time,
              lambda_factor);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "data: " << std::endl;
            lyfast::sysid::DifferentialUtils::printData(data, delta_time);
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
                std::vector<lyfast::sysid::DifferentialVoltageCommand> {
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
                std::vector<lyfast::sysid::DifferentialVoltageCommand> {
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

void linear_raw_ka_tuner(lyfast::KvUnits<LinearVelocity> left_Kv,
                         lyfast::KsUnits left_Ks,
                         lyfast::KvUnits<LinearVelocity> right_Kv,
                         lyfast::KsUnits right_Ks) {
    std::vector<lyfast::sysid::DifferentialVoltageCommand>
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

void angular_raw_ka_tuner(lyfast::KvUnits<LinearVelocity> left_Kv,
                          lyfast::KsUnits left_Ks,
                          lyfast::KvUnits<LinearVelocity> right_Kv,
                          lyfast::KsUnits right_Ks) {
    std::vector<lyfast::sysid::DifferentialVoltageCommand>
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

void path_follow_test() {
    using namespace blazing::lyfast;
    using namespace blazing::lyfast::geometry;
    using namespace blazing::lyfast::mp;
    arc_pose_tracker.setPose({ -23.6_in, 0_in, 0_stDeg });

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

    // effectively infinity
    AngularConstraints angular_constraints(2.0_radps, 1.3_radps2, 1.3_radps2);

    Constraints constraints(robot_constraints,
                            linear_constraints,
                            angular_constraints);

    // std::make_shared<geometry::Curve>(spline);

    std::shared_ptr<Trajectory> test_trajectory(
      new Trajectory(spline_ptr,
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
                     0_inps,
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
    lyfast::PathFollow(controllers, velocity_chassis, test_trajectory)
        .drive_toleranceDuration(100_sec)
        .drive_largeToleranceDuration(100_sec)
        // .parameterization(blazing::lyfast::time_based)
        .timeout(5_sec) |
      run;
}

pros::MotorGroup test_motor({ 13 }, pros::MotorGears::green);

// test motor kv ks and ka
auto Kv = 0.039394 * volt / radps;
auto Ks = 0.027303 * volt;
auto Ka = 0.001357 * volt / radps2;

auto motor_voltage_Kv = 0.039394 * volt / radps;
auto motor_voltage_Ks = 0.027303 * volt;
auto motor_voltage_Ka = 0.001357 * volt / radps2;

lyfast::EMAVelocityFilter::Constants ema_filter_constants {
    .final_gearing_rpm = 200_rpm,
    .Koffset = 0.1,
    .KalphaFactor = 0.7,
};
lyfast::EMAVelocityFilter ema_filter { &test_motor, ema_filter_constants };

lyfast::FeedforwardVelocityController<AngularVelocity> feedforward {
    lyfast::FeedforwardVelocityControllerParams<AngularVelocity> {
                                                                  .Kv = motor_voltage_Kv,
                                                                  .Ka = motor_voltage_Ka,
                                                                  .Ks = motor_voltage_Ks,
                                                                  }
};
lyfast::PIDVelocityController<AngularVelocity> feedback {
    lyfast::PIDVelocityControllerParams<AngularVelocity> {
                                                          .Kp = 0.01 * volt / radps,
                                                          .Ki = 0.10 * volt / rad,
                                                          .max_output = 1.0 * volt,
                                                          .tbh_factor = 0.0,
                                                          }
};
lyfast::SimpleVelocityController<AngularVelocity> controller { feedforward,
                                                               feedback };
lyfast::AngularMotorGroupVelocityPlant test_plant(&ema_filter, controller);

std::vector<FAngularVelocity> tick_based_vel_data;

std::vector<lyfast::sysid::AngularSysidEntry> raw_data, filtered_data;

std::vector<std::pair<FTorque, FCurrent>> extra_data;
std::vector<FPower> power_data;

void logInformation(Voltage commanded_voltage) {
    static uint32_t previousInternalMotorClock;
    static int32_t oldMotorTicks =
      test_motor.get_raw_position(&previousInternalMotorClock);

    uint32_t currentInternalMotorClock;
    int32_t currentMotorTicks =
      test_motor.get_raw_position(&currentInternalMotorClock);

    double dT =
      5.0 * std::round(
              (currentInternalMotorClock - previousInternalMotorClock) / 5.0);
    double dN = currentMotorTicks - oldMotorTicks;
    previousInternalMotorClock = currentInternalMotorClock;
    oldMotorTicks = currentMotorTicks;

    // 900 ticks / revolution
    Angle angular_position_delta = dN / (900 / rot);
    AngularVelocity estimated_angular_velocity =
      angular_position_delta / from_msec(dT);

    tick_based_vel_data.emplace_back(estimated_angular_velocity);

    Voltage filter_voltage = ema_filter.getInput();
    AngularVelocity filter_velocity = ema_filter.getPredictedState();

    AngularVelocity raw_vel = test_motor.get_actual_velocity() * rpm;

    Torque torque = test_motor.get_torque() * Nm; // calculated
    Current current = test_motor.get_current_draw() * mamp;
    Power power = test_motor.get_power() * watt;

    raw_data.emplace_back(raw_vel, commanded_voltage);
    filtered_data.emplace_back(filter_velocity, filter_voltage);
    extra_data.emplace_back(torque, current);
    power_data.emplace_back(power);
}

// task with critical timing that allows gathering consistent data
void timeCriticalTask() {
    // code section taken from sylib:
    // https://github.com/sy1vi3/sylib/blob/5b2eef6812f65b4305b74e415d3a570c10213fff/src/sylib/system.cpp

    // A 1ms loop will actually take around 1040 or 960 microseconds, always
    // alternating. Over 3ms, the total length of time in micros should be
    // either around 3040 or 960 Daemon needs to start on a cycle to be
    // directly opposite of vexBackgroundProcessing()
    // vexBackgroundProcessing always runs after a short cycle, meaning the
    // sylib daemon needs to start after a long cycle Values offset by 20 to
    // give room for error, the groupings are very tight so it shouldnt
    // matter

    constexpr std::uint64_t LONG_MICROS_CYCLE_LENGTH = 1040 - 20;
    constexpr std::uint64_t AVERAGE_MICROS_CYCLE_LENGTH = 1000;
    constexpr std::uint64_t DIFFERENCE_BETWEEN_AVERAGE_AND_LONG =
      LONG_MICROS_CYCLE_LENGTH - AVERAGE_MICROS_CYCLE_LENGTH;

    uint32_t systemTime = pros::millis();
    uint32_t detectorPreviousTime = pros::millis();
    uint64_t systemTimeMicros = pros::micros();
    uint64_t prevMicros = systemTimeMicros;

    int frameCount = 0;

    do {
        systemTimeMicros = pros::micros();
        detectorPreviousTime = systemTime;
        prevMicros = systemTimeMicros;
        pros::Task::delay_until(&systemTime, 3);
    } while (
      (pros::micros() - prevMicros) >
      (((systemTime - detectorPreviousTime) * AVERAGE_MICROS_CYCLE_LENGTH) -
       DIFFERENCE_BETWEEN_AVERAGE_AND_LONG));
    /*
    NOW WE'RE TIMED CORRECTLY, STARTING DAEMON
    */

    while (1) {
        {
            frameCount++;

            // do stuff here
            if (frameCount % 5 == 0) {
                uint32_t curr_time = pros::millis();

                // predict the filter
                ema_filter.predictToTimestamp(curr_time);
                ema_filter.correct();
                // update plant
                test_plant.updateToTimestamp(curr_time);

                Voltage test_plant_voltage = test_plant.getCommandedVoltage();

                // actuate motor
                test_motor.move_voltage(12 * to_mvolt(test_plant_voltage));

                // drivetrain related things
                left_ema_filter.predictToTimestamp(curr_time);
                left_ema_filter.correct();

                right_ema_filter.predictToTimestamp(curr_time);
                right_ema_filter.correct();

                drivetrain_plant.updateToTimestamp(curr_time);
                auto curr_drivetrain_voltages =
                  drivetrain_plant.getCommandedVoltages();

                // actuate motors with desired voltages
                left_motors.move_voltage(
                  12 * to_mvolt(curr_drivetrain_voltages.left_voltage));
                right_motors.move_voltage(
                  12 * to_mvolt(curr_drivetrain_voltages.right_voltage));

                logInformation(test_plant_voltage);
            }

            pros::Task::delay_until(&systemTime, 2);
        }
    }
}

void startTimeCriticalTask() {
    static bool daemonStarted = false;
    if (!daemonStarted) {
        pros::Task managerTask(timeCriticalTask,
                               15,
                               TASK_STACK_DEPTH_DEFAULT,
                               "time critical task");
        daemonStarted = true;
    }
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

    startTimeCriticalTask();
}

void test_motor_kv_ks_tuner() {
    using namespace lyfast::sysid;
    std::vector<VoltageCommand> kv_ks_commands = {
        // linear movements
        { -0.05_volt, 600_msec },
        { 0.05_volt,  600_msec },
        { -0.1_volt,  800_msec },
        { 0.1_volt,   800_msec },
        { -0.2_volt,  800_msec },
        { 0.2_volt,   800_msec },
        { -0.3_volt,  800_msec },
        { 0.3_volt,   800_msec },
        { -0.4_volt,  800_msec },
        { 0.4_volt,   800_msec },
        { -0.5_volt,  800_msec },
        { 0.5_volt,   800_msec },
        { -0.6_volt,  800_msec },
        { 0.6_volt,   800_msec },
        { -0.7_volt,  800_msec },
        { 0.7_volt,   800_msec },
        { -0.8_volt,  800_msec },
        { 0.8_volt,   800_msec },
        { -0.9_volt,  800_msec },
        { 0.9_volt,   800_msec },
        { -1.0_volt,  800_msec },
        { 1.0_volt,   800_msec },
    };

    AngularVelocity final_rpm = 200_rpm;

    auto vel_func = [&]() -> AngularVelocity {
        // return get_group_velocity(&test_motor, final_rpm);
        // can use filtered data for kv/ks
        return ema_filter.getPredictedState();
    };
    auto voltage_func = [&](Voltage commanded_voltage) -> Voltage {
        // using either will return different values?
        // return commanded_voltage;
        return get_group_voltage(&test_motor);
    };

    auto data =
      AngularMotorGroupUtils::generateData(kv_ks_commands,
                                           &test_motor,
                                           vel_func,
                                           voltage_func,
                                           180_msec, // takes 20 samples?
                                           10_msec);

    std::cout << "Data: " << std::endl;
    AngularMotorGroupUtils::printDataAsLatex(data);

    auto [Kv, Ks] = AngularMotorGroupUtils::fit_kv_ks_data(data);

    std::cout << "Kv/Ks: " << Kv.convert(volt / radps) << " "
              << Ks.convert(volt) << std::endl;
}

void test_motor_ka_tuner() {
    using namespace lyfast::sysid;
    std::vector<VoltageCommand> ka_commands = {
        // bunch of harsh accelerations
        { -0.05_volt, 200_msec },
        { 0.05_volt,  200_msec },
        { -0.2_volt,  200_msec },
        { 0.2_volt,   200_msec },
        { -0.1_volt,  200_msec },
        { 0.1_volt,   200_msec },
        { -0.3_volt,  200_msec },
        { 0.3_volt,   200_msec },
        { -0.5_volt,  200_msec },
        { 0.5_volt,   200_msec },
        { -0.4_volt,  200_msec },
        { 0.4_volt,   200_msec },
        { -0.6_volt,  200_msec },
        { 0.6_volt,   200_msec },
        { -0.8_volt,  200_msec },
        { 0.8_volt,   200_msec },
        { -0.7_volt,  200_msec },
        { 0.7_volt,   200_msec },
        { -0.9_volt,  200_msec },
        { 0.9_volt,   200_msec },
        { -1.0_volt,  200_msec },
        { 1.0_volt,   200_msec },

        // gradual up and down
        // { -0.05_volt, 100_msec },
        { -0.10_volt, 50_msec  },
        // { -0.15_volt, 100_msec },
        { -0.20_volt, 50_msec  },
        // { -0.25_volt, 100_msec },
        { -0.30_volt, 50_msec  },
        // { -0.35_volt, 100_msec },
        { -0.40_volt, 50_msec  },
        // { -0.45_volt, 100_msec },
        { -0.50_volt, 50_msec  },
        // { -0.55_volt, 100_msec },
        { -0.60_volt, 50_msec  },
        // { -0.65_volt, 100_msec },
        { -0.70_volt, 50_msec  },
        // { -0.75_volt, 100_msec },
        { -0.80_volt, 60_msec  },
        // { -0.85_volt, 100_msec },
        { -0.90_volt, 60_msec  },
        // { -0.95_volt, 100_msec },
        { -1.00_volt, 60_msec  },
        { -1.00_volt,
         100_msec              }, // more gradual up and downs in other direction
        { -0.90_volt, 50_msec  },
        { -0.80_volt, 50_msec  },
        { -0.70_volt, 50_msec  },
        { -0.60_volt, 50_msec  },
        { -0.50_volt, 50_msec  },
        { -0.40_volt, 50_msec  },
        { -0.30_volt, 50_msec  },
        { -0.20_volt, 50_msec  },
        { -0.10_volt, 50_msec  },
        { -0.05_volt, 50_msec  },

        { 0.05_volt,  25_msec  },
        { 0.10_volt,  25_msec  },
        { 0.15_volt,  25_msec  },
        { 0.20_volt,  25_msec  },
        { 0.25_volt,  25_msec  },
        { 0.30_volt,  25_msec  },
        { 0.35_volt,  25_msec  },
        { 0.40_volt,  25_msec  },
        { 0.45_volt,  25_msec  },
        { 0.50_volt,  25_msec  },
        { 0.55_volt,  25_msec  },
        { 0.60_volt,  25_msec  },
        { 0.65_volt,  25_msec  },
        { 0.70_volt,  25_msec  },
        { 0.75_volt,  25_msec  },
        { 0.80_volt,  25_msec  },
        { 0.85_volt,  25_msec  },
        { 0.90_volt,  25_msec  },
        { 0.95_volt,  25_msec  },
        { 1.00_volt,  25_msec  },
        { 1.00_volt,  25_msec  },
        { 0.95_volt,  25_msec  },
        { 0.90_volt,  25_msec  },
        { 0.85_volt,  25_msec  },
        { 0.80_volt,  25_msec  },
        { 0.75_volt,  25_msec  },
        { 0.70_volt,  25_msec  },
        { 0.65_volt,  25_msec  },
        { 0.60_volt,  25_msec  },
        { 0.55_volt,  25_msec  },
        { 0.50_volt,  25_msec  },
        { 0.45_volt,  25_msec  },
        { 0.40_volt,  25_msec  },
        { 0.35_volt,  25_msec  },
        { 0.30_volt,  25_msec  },
        { 0.25_volt,  25_msec  },
        { 0.20_volt,  25_msec  },
        { 0.15_volt,  25_msec  },
        { 0.10_volt,  25_msec  },
        { 0.05_volt,  25_msec  },
    };

    AngularVelocity final_rpm = 200_rpm;

    auto vel_func = [&]() -> AngularVelocity {
        return get_group_velocity(&test_motor, final_rpm);
    };
    auto voltage_func = [&](Voltage commanded_voltage) -> Voltage {
        // using either will return different values?
        // return commanded_voltage;
        return get_group_voltage(&test_motor);
    };

    auto data = AngularMotorGroupUtils::generateData(ka_commands,
                                                     &test_motor,
                                                     vel_func,
                                                     voltage_func,
                                                     std::nullopt,
                                                     10_msec);

    std::cout << "got data!" << std::endl;

    std::cout << "ka Data: " << std::endl;
    AngularMotorGroupUtils::printDataAsLatex(data);

    std::cout << "trying Ka_method1: " << std::endl;

    auto Ka_method1 = AngularMotorGroupUtils::fit_ka_data(data,
                                                          10_msec,
                                                          motor_voltage_Kv,
                                                          motor_voltage_Ks);
    std::cout << "Ka_method1: " << Ka_method1.convert(volt / radps2)
              << std::endl;
}

void motorPlantTest() {

    using namespace lyfast::sysid;

    // test_motor_kv_ks_tuner();
    // test_motor_ka_tuner();
    // return;

    std::vector<std::pair<AngularVelocity, Time>> test_commands = {
        { 0.1 * 200_rpm,  400_msec  },
        { 0.3 * 200_rpm,  500_msec  },
        { 0.4 * 200_rpm,  100_msec  },
        { 0.2 * 200_rpm,  100_msec  },
        { -0.5 * 200_rpm, 600_msec  },
        { -1.0 * 200_rpm, 600_msec  },
        { 1.0 * 200_rpm,  1000_msec },
    };

    //
    // std::vector<VoltageCommand> test_commands = {
    //     { 0.1_volt,  400_msec  },
    //     { 0.3_volt,  500_msec  },
    //     { 0.4_volt,  100_msec  },
    //     { 0.2_volt,  100_msec  },
    //     { -0.5_volt, 600_msec  },
    //     { -1.0_volt, 600_msec  },
    //     { 1.0_volt,  1000_msec },
    // };
    //
    // for (auto [voltage, duration, record] : test_commands) {
    //     test_motor.move_voltage(12 * to_mvolt(voltage));
    //
    //     pros::delay(to_msec(duration));
    // }
    //
    for (auto [velocity, duration] : test_commands) {
        test_plant.setTarget(velocity);

        pros::delay(to_msec(duration));
    }

    std::cout << "raw data: " << std::endl;
    AngularMotorGroupUtils::printDataAsLatex(raw_data);

    std::cout << "filtered data: " << std::endl;
    AngularMotorGroupUtils::printDataAsLatex(filtered_data);

    std::cout << "extra data (torque, current): " << std::endl;
    printPairQuantitiesAsLatex(extra_data);

    std::cout << "power data: " << std::endl;
    printQuantityVectorAsLatex(power_data);
    std::cout << "tick based vel data: " << std::endl;
    printQuantityVectorAsLatex(tick_based_vel_data);
}

void opcontrol() {
    // pros::delay(2000);
    motorPlantTest();
}
