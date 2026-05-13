#include "main.h"
#include "blazing/api.hpp"
#include "blazing/latex_utils.hpp"
#include "blazing/tolerances.hpp"
#include "blazing/utils.hpp"
#include "liblvgl/llemu.hpp"
#include "lyfast/api.hpp"
#include "lyfast/controllers/path_pose_feedback.hpp"
#include "lyfast/drivetrains/velocity_differential.hpp"
#include "lyfast/motion_profiling/mp.hpp"
#include "lyfast/plants/velocity_plants.hpp"
#include "lyfast/sysid/system_identification.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/apix.h"
#include "pros/imu.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
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
#include <optional>
#include <tuple>
#include <utility>
#include <variant>

void disabled() {}

void competition_initialize() {}

void autonomous() {}

namespace controls {
const auto L1 = pros::E_CONTROLLER_DIGITAL_L1;
const auto R1 = pros::E_CONTROLLER_DIGITAL_R1;

const auto L2 = pros::E_CONTROLLER_DIGITAL_L2;
const auto R2 = pros::E_CONTROLLER_DIGITAL_R2;

const auto X = pros::E_CONTROLLER_DIGITAL_X;
const auto Y = pros::E_CONTROLLER_DIGITAL_Y;
const auto A = pros::E_CONTROLLER_DIGITAL_A;
const auto B = pros::E_CONTROLLER_DIGITAL_B;

const auto UP = pros::E_CONTROLLER_DIGITAL_UP;
const auto DOWN = pros::E_CONTROLLER_DIGITAL_DOWN;
const auto LEFT = pros::E_CONTROLLER_DIGITAL_LEFT;
const auto RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT;

const auto LEFT_SHIFT = pros::E_CONTROLLER_DIGITAL_RIGHT;
const auto RIGHT_SHIFT = pros::E_CONTROLLER_DIGITAL_Y;
} // namespace controls

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

pros::MotorGroup left_motors({ left_front, left_middle, left_back }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::MotorGroup right_motors({ right_front, right_middle, right_back }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
// clang-format on

ScaledIMU imu(20, (360.0 + 1.5) / 360.0);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// odom rotation sensors
// pros::Rotation forwards_odom_rotation(-20);
pros::Rotation forwards_odom_rotation(-13);
pros::Rotation sideways_odom_rotation(18);

using namespace blazing;

Length track_width = 10.5_in;
Length wheel_diameter = 3.25_in;
AngularVelocity final_rpm = 450_rpm;

// tracker stuff
// DifferentialDrivetrain
//   drivetrain(&left_motors, &right_motors, wheel_diameter, final_rpm);

lyfast::DifferentialVelocityControllerParams vel_controller_params {
	.linear = {
		// TODO: recalc angular?
		// .left_Kv = 0.46 * volt / mps,
		.left_Kv = 0.46 * volt / mps,
		.left_Ka = 0.09 * volt / mps2,
		.left_low_target_Kv = 0.4 * volt / mps,
		.left_low_target_Ka = 0.04 * volt / mps2,
		// .left_Ks = 0.08 * volt,
		.left_Ks = 0.04 * volt,

		// .right_Kv = 0.49 * volt / mps,
		.right_Kv = 0.47 * volt / mps,
		.right_Ka = 0.09 * volt / mps2,
		.right_low_target_Kv = 0.4 * volt / mps,
		.right_low_target_Ka = 0.04 * volt / mps2,
		// .right_Ks = 0.08 * volt,
		.right_Ks = 0.04 * volt,

		.Ka_delta_time = 20_msec,
		.low_target_threshold = 20_inps
	},
	.angular = {
		.left_Kv = 0.90 * volt / mps,
		// .left_Ka = 0.11 * volt / mps2,
		.left_Ka = 0.07 * volt / mps2,
		.left_low_target_Kv = 0.5 * volt / mps,
		.left_low_target_Ka = 0.0 * volt / mps2,
		.left_Ks = 0.08 * volt,

		.right_Kv = 0.90 * volt / mps,
		// .right_Ka = 0.11 * volt / mps2,
		.right_Ka = 0.07 * volt / mps2,
		.right_low_target_Kv = 0.5 * volt / mps,
		.right_low_target_Ka = 0.0 * volt / mps2,
		.right_Ks = 0.08 * volt,

		.Ka_delta_time = 20_msec,
		.low_target_threshold = 2_inps
	},
	.linear_pid = {
		.left_Kp = 0.5 * volt / mps,
		.left_Kp_close = 0.0 * volt / mps,
		.left_Kp_low = 0.0 * volt / mps,
		.left_low_threshold = 7_inps,
		.left_close_threshold = 0_inps,
		// .left_Ki = 1.0 * volt / m,
		.left_Ki = 0.0 * volt / m,
		.left_Ki_windup = 12_inps,
		//
		.left_max_output =  1_volt,
		.left_tbh_factor =  1.0,

		.right_Kp = 0.5 * volt / mps,
		.right_Kp_close = 0.0 * volt / mps,
		.right_Kp_low = 0.0 * volt / mps,
		.right_low_threshold = 7_inps,
		.right_close_threshold = 0_inps,
		// .right_Ki = 1.0 * volt / m,
		.right_Ki = 0.0 * volt / m,
		.right_Ki_windup = 12_inps,

		.right_max_output =  1_volt,
		.right_tbh_factor =  1.0,


		// .left_Kp = 1.5 * volt / mps,
		// .left_Kp_close = 0.0 * volt / mps,
		// .left_Kp_low = 0.0 * volt / mps,
		// .left_low_threshold = 7_inps,
		// .left_close_threshold = 0_inps,
		// // .left_Ki = 1.0 * volt / m,
		// .left_Ki = 0.0 * volt / m,
		// .left_Ki_windup = 12_inps,
		// //
		// .left_max_output =  1_volt,
		// .left_tbh_factor =  1.0,
		//
		// .right_Kp = 1.5 * volt / mps,
		// .right_Kp_close = 0.0 * volt / mps,
		// .right_Kp_low = 0.0 * volt / mps,
		// .right_low_threshold = 7_inps,
		// .right_close_threshold = 0_inps,
		// // .right_Ki = 1.0 * volt / m,
		// .right_Ki = 0.0 * volt / m,
		// .right_Ki_windup = 12_inps,
		//
		// .right_max_output =  1_volt,
		// .right_tbh_factor =  1.0,
	},
	.angular_pid = {
		// .left_Kp = 1.5 * volt / mps,
		// .left_Kp_close = 0.0 * volt / mps,
		// .left_Kp_low = 0.0 * volt / mps,
		// .left_low_threshold = 10_inps,
		// .left_close_threshold = 0_inps,
		// // .left_Ki = 1.5 * volt / m,
		// .left_Ki = 0.0 * volt / m,
		// .left_Ki_windup = 12_inps,
		// //
		// .left_max_output =  1_volt,
		// .left_tbh_factor =  1.0,
		//
		// .right_Kp = 1.5 * volt / mps,
		// .right_Kp_close = 0.0 * volt / mps,
		// .right_Kp_low = 0.0 * volt / mps,
		// .right_low_threshold = 10_inps,
		// .right_close_threshold = 0_inps,
		// // .right_Ki = 1.5 * volt / m,
		// .right_Ki = 0.0 * volt / m,
		// .right_Ki_windup = 12_inps,
		//
		// .right_max_output =  1_volt,
		// .right_tbh_factor =  1.0,
	}
};
lyfast::DifferentialVelocityController vel_controller { vel_controller_params,
                                                        76_inps,
                                                        track_width,
                                                        false };

lyfast::EMAVelocityFilter::Constants drivetrain_ema_filter_constants {
    .final_gearing_rpm = 450_rpm,
    .Koffset = 0.15, // guaranteed to trust velocity measurements always
    // .Koffset = 0.1,
    // .KalphaFactor = 0.1,
    .KalphaFactor = 0.005,
};

// left back motor ime doesnt work well
pros::MotorGroup ema_left_motors({ left_front, left_middle },
                                 pros::MotorGears::blue,
                                 pros::MotorEncoderUnits::rotations);

lyfast::EMAVelocityFilter left_ema_filter { &ema_left_motors,
                                            drivetrain_ema_filter_constants };
lyfast::EMAVelocityFilter right_ema_filter { &right_motors,
                                             drivetrain_ema_filter_constants };

lyfast::DrivetrainVelocityPlant drivetrain_plant { &left_ema_filter,
                                                   &right_ema_filter,
                                                   vel_controller,
                                                   wheel_diameter };

lyfast::VelocityDifferentialDrivetrain
  drivetrain(&left_motors, &right_motors, &drivetrain_plant, track_width);

ForwardsTracker
  left_motor_tracker(&left_motors, -track_width / 2, wheel_diameter, final_rpm);

ForwardsTracker right_motor_tracker(&right_motors,
                                    track_width / 2,
                                    wheel_diameter,
                                    final_rpm);

// TODO: update since now sideways might be zero
ForwardsTracker forwards_tracker(&forwards_odom_rotation, 0.04_in, 1.991_in);
SidewaysTracker sideways_tracker(&sideways_odom_rotation, -2.1_in, 1.991_in);

TrackingImu tracking_imu(&imu);

ArcOdomTracker arc_pose_tracker({ &forwards_tracker,
                                  &left_motor_tracker,
                                  &right_motor_tracker },
                                { &sideways_tracker },
                                // {},
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
Tolerances linearTolerances(100_msec,
                            ErrorTolerance { 0.7_in },
                            VelocityTolerance { 400_inps }
);

Tolerances angularTolerances(40_msec,
                             ErrorTolerance { 2.25_stDeg },
                             VelocityTolerance { 120_degps });

// large tolerances
// Tolerances largeLinearTolerances(400_msec,
//                                  ErrorTolerance { 3_in },
//                                  VelocityTolerance { 30_inps });
Tolerances largeLinearTolerances(400_sec,
                                 ErrorTolerance { 3_in },
                                 VelocityTolerance { 30_inps });

Tolerances largeAngularTolerances(400_msec,
                                  ErrorTolerance { 7_stDeg },
                                  VelocityTolerance { 70_inps });

// chain tolerances
Tolerances chainLinearTolerances(1_sec, ErrorTolerance { 6_in });
Tolerances chainAngularTolerances(1_sec, ErrorTolerance { 20_stDeg });

normalLargeChainTolerances tolerances(linearTolerances,
                                      angularTolerances,
                                      largeLinearTolerances,
                                      largeAngularTolerances,

                                      chainLinearTolerances,
                                      chainAngularTolerances);

// Chassis chassis(drivetrain, arc_pose_tracker, tolerances);
Chassis velocity_chassis(&drivetrain, &arc_pose_tracker, tolerances);

RunExecutor run;
AsyncExecutor async;

FLength track_radius = track_width * 0.5;

FLinearVelocity max_velocity = 76_Finps;
// w = v / r
FAngularVelocity max_angular_velocity = (max_velocity / track_radius) * Frad;

std::array<float, 3> Q { (40_in).internal(),
                         // (6_in).internal(),
                         // (5_in).internal(),
                         (8_in).internal(),
                         // (1_in).internal(),
                         // (5_stDeg).internal() };
                         (180_stDeg).internal() };

// [x, theta]
std::array<float, 2> simple_Q { (10000_in).internal(),
                                (50000_stDeg).internal() };

std::array<float, 2> simple_R { (1_inps).internal(),
                                // max angular velocity
                                (1_degps).internal() };

std::array<float, 2> R { // max velocity
                         max_velocity.internal(),
                         // max angular velocity
                         max_angular_velocity.internal()
};

Time input_delay = 40_msec;
LinearVelocity lqr_minimum_velocity = 1.0_inps;

blazing::lyfast::state_space::LTVUnicycleController
  lqr_controller(Q, R, simple_Q, simple_R, 0_msec, lqr_minimum_velocity);

// blazing::lyfast::RamsetteController ramsete_controller(1, 0.5);
// blazing::lyfast::NoPathFeedbackController no_feedback_controller;

lyfast::PathPoseFeedbackController<decltype(lqr_controller)>
  path_pose_feedback_controller(lqr_controller);
// lyfast::PathPoseFeedbackController<decltype(ramsete_controller)>
// path_pose_feedback_controller(ramsete_controller);
// lyfast::PathPoseFeedbackController<decltype(no_feedback_controller)>
// path_pose_feedback_controller(no_feedback_controller);

PID<Angle, AngularVelocity>
  linear_angular_vel_pid(12.50,
                         0.0,
                         8.0,
                         to_stRad(10_stDeg), // windup range
                         76, // restrict max vel
                         std::nullopt, // derivative alpha
                         50_msec,
                         1_stRad,
                         1_radps);

PID<Angle, AngularVelocity> turn_heading_vel_pid(
  10.100,
  // 0.01,
  0.0,
  0.600,
  // 19.000,
  //                       // 0.01,
  //                       0.0,
  //                       19.050,
  to_stRad(10_stDeg),
  // std::nullopt,
  // to_radps(rad * 76_inps / (10.5_in * 0.5)), // max speed
  to_radps(max_angular_velocity),
  std::nullopt, // derivative alpha
  50_msec,
  1_stRad,
  1_radps);

PIDAngularVelocityController angular_vel_pid_controller(linear_angular_vel_pid);

AngularVelocitySlewController angular_vel_slew_controller {};
AngularVelocityClampController angular_vel_clamp_controller {};

LinearSlewController linear_slew { std::nullopt, 0.2_volt };
// AngularSlewController angular_slew(0.8_volt);
AngularSlewController angular_slew {};

lyfast::mpFeedback<Length>
  linear_mp_feedback(70_inps, 110_inps2, 0.3_in, 0.05_inps / 0.20_in);

LinearVelocityFeedbackController<decltype(linear_mp_feedback)>
  linear_mp_feedback_controller(linear_mp_feedback);

// simulates accel
LinearVelocitySlewController linear_vel_slew_controller { 170_inps2 };
// LinearVelocitySlewController linear_vel_slew_controller {};
LinearVelocityClampController linear_vel_clamp_controller {};

LinearVoltageClampController linear_voltage_constraints;
AngularVoltageClampController angular_voltage_constraints;

Controllers controllers(
  // pid controllers
  // PIDLinearController(linear_pid),
  // PIDAngularController(angular_pid),
  //
  // path_pose_feedback_controller,
  //
  // LinearSlewController {},
  // AngularSlewController {},
  //
  // // voltage constraints controllers
  // LinearVoltageClampController(),
  // AngularVoltageClampController());

  // pid controllers
  PIDLinearController(linear_pid),
  PIDAngularController(angular_pid),
  path_pose_feedback_controller,

  // slew controllers
  linear_slew,
  angular_slew,

  linear_mp_feedback_controller,
  linear_vel_slew_controller,
  linear_vel_clamp_controller,

  // angular velocity controllers
  angular_vel_pid_controller,
  angular_vel_slew_controller,
  angular_vel_clamp_controller,

  // voltage constraints controllers
  // (included just so they can be set per motion)
  linear_voltage_constraints,
  angular_voltage_constraints);

// normal tolerances

// .large_duration = 400_sec,
//   .large_error { 3_in }, .large_velocity { 30_inps },
//
//   .chain_duration = 1_sec, .chain_error { 6_in },

// avoids a division by zero
AsyncExecutor chain;

// custom cos-like func
double angular_linear_func(Angle angle) {
    // reduces the domain to [0,pi]
    angle = units::abs(units::constrainAngle180(angle));

    // double sgn = units::sgn(angle);
    // angle = units::abs(angle);

    // defined on the range [0,pi/2]
    // auto func = [](double x) -> double {
    //     double poly = 0.0001;
    //     if (x < 1.224747) {
    //         // simple polynomial that delays linear output until angle error
    //         is small poly = 1.0 - 2.0 * (x * x) + 1.08866 * (x * x * x);
    //     }
    //     // return 0.00001;
    //     return 0.7 * poly + std::cos(x) * 0.3;
    // };

    // defined on the range [0,pi/2]
    // auto func = [](double x) -> double {
    //     return std::exp(-1.25 * x);
    // };
    //
    // defined on the range [0,pi/2]
    auto func = [](double x) -> double {
        // double a = 0.93;
        double a = 0.5;
        return std::exp(-a * x) * (1 - (2 / M_PI) * x);
    };

    // makes this function apply on the range [0,pi]
    if (angle <= rot / 4.0) {
        return func(angle.internal());
    } else {
        return -func(M_PI - angle.internal());
    }
};

// MotionBuilder mb(chassis, controllers);
MotionBuilder mb(velocity_chassis, controllers);

std::vector<std::array<FLinearVelocity, 3>> left_tick_velocity;
std::vector<std::array<FLinearVelocity, 3>> right_tick_velocity;
std::vector<FDifferentialSpeeds> target_velocities;
std::vector<FVoltage> left_target_voltage;
std::vector<FVoltage> right_target_voltage;

std::vector<lyfast::sysid::LinearSysidEntry> left_filtered_data,
  right_filtered_data;
bool drivetrain_tick_logging = false;

void printDrivetrainData() {
    // only print data if logging is enabled
    if (drivetrain_tick_logging) {
        // disable so data doesnt get logged as data is being printed
        drivetrain_tick_logging = false;
        std::cout << "targets: " << std::endl;
        printPairListAsLatex(
          size(target_velocities),
          [&](size_t idx) -> std::pair<float, float> {
              return { target_velocities[idx].linear_velocity.internal(),
                       target_velocities[idx].angular_velocity.internal() };
          });

        std::cout << "left target voltage:" << std::endl;
        printQuantityVectorAsLatex(left_target_voltage);
        std::cout << "right target voltage:" << std::endl;
        printQuantityVectorAsLatex(right_target_voltage);

        std::cout << "left filtered data: " << std::endl;
        lyfast::sysid::LinearMotorGroupUtils::printDataAsLatex(
          left_filtered_data);

        std::cout << "right filtered data: " << std::endl;
        lyfast::sysid::LinearMotorGroupUtils::printDataAsLatex(
          right_filtered_data);

        std::cout << "left tick data: " << std::endl;
        for (int i = 0; i < 3; i++)
            printListAsLatex(size(left_tick_velocity),
                             [&](size_t idx) -> float {
                                 return left_tick_velocity[idx][i].convert(mps);
                             });

        std::cout << "right tick data: " << std::endl;
        for (int i = 0; i < 3; i++)
            printListAsLatex(size(right_tick_velocity),
                             [&](size_t idx) -> float {
                                 return right_tick_velocity[idx][i].convert(
                                   mps);
                             });

        // enable again in case more data is gonna be collected
        drivetrain_tick_logging = true;
    }
}

void trajectoryDebugPrint(const lyfast::mp::Trajectory* trajectory) {
    using namespace blazing::lyfast;
    using namespace blazing::lyfast::geometry;
    using namespace blazing::lyfast::mp;
    auto print =
      []<typename Unit>(std::string name,
                        const std::vector<Trajectory::debugInfo>& list,
                        Unit Trajectory::debugInfo::* member,
                        Unit target_units) {
          std::cout << name << "=\\left[";
          for (size_t i = 0; i < list.size(); i++) {
              if (i != 0) std::cout << ",";
              std::cout << (list[i].*member).convert(target_units);
          }
          std::cout << "\\right]" << std::endl;
      };

    if (trajectory->getDebugEnabled()) {
        std::cout << std::fixed << std::setprecision(4);
        print("a_{kin}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::max_kin_decel,
              Finps2);
        print("a_{turn}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::max_turn_accel,
              Finps2);
        //
        print("d_{kin}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::max_kin_decel,
              Finps2);
        print("d_{turn}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::max_turn_decel,
              Finps2);
        //
        print("v_{kin}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::max_kin_vel,
              Finps);
        print("v_{turn}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::max_turn_vel,
              Finps);
        //
        print("v_{friction}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::max_friction_vel,
              Finps);

        print("v_{forward}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::forwards_pass,
              Finps);
        print("v_{backward}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::backwards_pass,
              Finps);

        print("v_{final}",
              trajectory->getDebugInfo(),
              &Trajectory::debugInfo::final_vels,
              Finps);

        std::cout << "l_{times}=\\left[";
        for (auto& point : trajectory->getPoints()) {
            std::cout << point.travel_time.convert(sec) << ",";
        }
        std::cout << "\\right]" << std::endl;

        std::cout << "l_{points}=\\left[";
        for (auto& point : trajectory->getPoints()) {
            std::cout << "\\left(" << point.point.x.convert(in) << ","
                      << point.point.y.convert(in) << "\\right),";
        }
        std::cout << "\\right]" << std::endl;

        std::cout << "l_{headings}=\\left[";
        for (auto& point : trajectory->getPoints()) {
            std::cout << point.heading.internal() << ",";
        }
        std::cout << "\\right]" << std::endl;
    }
}

std::shared_ptr<lyfast::geometry::Line>
line(float x0, float y0, float x1, float y1) {
    return std::make_shared<lyfast::geometry::Line>(
      units::V2FPosition { from_in(x0), from_in(y0) },
      units::V2FPosition { from_in(x1), from_in(y1) });
}

std::shared_ptr<lyfast::geometry::CubicBezier> curve(float x0,
                                                     float y0,
                                                     float x1,
                                                     float y1,
                                                     float x2,
                                                     float y2,
                                                     float x3,
                                                     float y3) {
    return std::make_shared<lyfast::geometry::CubicBezier>(
      units::V2FPosition { from_in(x0), from_in(y0) },
      units::V2FPosition { from_in(x1), from_in(y1) },
      units::V2FPosition { from_in(x2), from_in(y2) },
      units::V2FPosition { from_in(x3), from_in(y3) });
}

namespace skills_paths {
auto start_TO_in_red_park = line(-44.125, 0.039, -61.257, 0.363);
auto in_red_park_TO_out_of_red = line(-61.257, 0.363, -44.365, 0.235);
auto out_of_red_TO_get_blue_middle =
  curve(-44.365, 0.235, -30.572, -0.922, -16.487, 8.767, -17.78, 17.656);
auto get_blue_middle_TO_score_middle = line(-17.78, 17.656, -13.179, 12.513);
auto score_middle_TO_ull =
  curve(-13.179, 12.513, -30.308, 27.086, -32.253, 47.2, -37.891, 47.2);
auto ull_TO_uls = line(-37.891, 47.2, -30.06, 47.2);
auto uls_TO_ulm = line(-30.06, 47.2, -57.173, 46.6);
auto ulm_TO_url1 =
  curve(-57.173, 46.6, -37.078, 46.6, -49.792, 67.874, 22.979, 59.339);
auto url1_TO_End_Control =
  curve(22.979, 59.339, 30.56, 58.034, 34.696, 53.895, 36, 47.2);
auto End_Control_TO_urls = line(36, 47.2, 30.2, 47.2);
auto urls_TO_urm = line(30.2, 47.2, 56.959, 46.6);
auto urm_TO_urls2 = line(56.959, 46.6, 30.2, 47.2);
auto urls2_TO_ur_cluster =
  curve(30.2, 47.2, 40.083, 46.256, 27.22, 37.467, 30.776, 31.164);
auto ur_cluster_TO_blue_park = line(30.776, 31.164, 44.859, -0.234);
auto blue_park_TO_in_blue_park = line(44.859, -0.234, 61.945, -0.234);
auto in_blue_park_TO_blue_park2 = line(61.945, -0.234, 45.304, -0.056);
auto blue_park2_TO_go_bottom = line(45.304, -0.056, 16.95, 18.346);
auto go_bottom_TO_bottom_score = line(16.95, 18.346, 11.967, 12.295);
auto bottom_score_TO_back_bottom = line(11.967, 12.295, 16.594, 16.389);
auto back_bottom_TO_dr_cluster = line(16.594, 16.389, 23.741, -23.427);
auto dr_cluster_TO_drl =
  curve(23.741, -23.427, 36.435, -44.03, 35.082, -47.2, 39.523, -47.2);
auto drl_TO_drls = line(39.523, -47.2, 29.949, -47.2);
auto drls_TO_drm = line(29.949, -47.2, 57.098, -47.059);
auto drm_TO_dll =
  curve(57.098, -47.059, 25.566, -46.6, 48.236, -64.96, -22.032, -60.53);
auto dll_TO_dls =
  curve(-22.032, -60.53, -37.44, -60.465, -45.449, -47.829, -31.435, -47.2);
auto dls_TO_dlm = line(-31.435, -47.2, -56.502, -47.295);
auto dlm_TO_dls2 = line(-56.502, -47.295, -31.435, -47.2);
auto dls2_TO_ending =
  curve(-31.435, -47.2, -66.497, -47.2, -61.239, -20.894, -62.307, -0.693);
} // namespace skills_paths

// void path_follow_test() {
//     using namespace blazing::lyfast;
//     using namespace blazing::lyfast::geometry;
//     using namespace blazing::lyfast::mp;
//     //
//     std::shared_ptr<Line> line(new Line({ -23.6_in, 0_in }, { 0_in, 0_in }));
//     std::shared_ptr<CubicBezier> bezier(new CubicBezier({ 0_in, 0_in },
//                                                         { 23.6_in, 0_in },
//                                                         { 23.6_in, 23.6_in },
//                                                         { 47.2_in, 23.6_in
//                                                         }));
//
//     // std::shared_ptr<geometry::Spline> spline_ptr { new Spline(
//     //   { line, bezier }) };
//
//     // auto spline_ptr = skills_paths::ulm_TO_url1;
//
//     std::shared_ptr<geometry::Spline> spline_ptr { new Spline(
//       { skills_paths::ulm_TO_url1, skills_paths::url1_TO_End_Control }) };
//
//     auto start_position = spline_ptr->getFirstEndpoint();
//     auto start_angle = spline_ptr->df(0).getAngle();
//     arc_pose_tracker.setPose({ start_position, start_angle });
//
//     RobotConstraints robot_constraints(
//       10.5_in, // track with
//       // 0.05, // friction coeff - should tune?
//       1.00, // friction coeff - should tune?
//       3.25_in, // wheel diameter
//       389_rpm, // max ang vel - determined somewhat from data
//       6.7_kg, // about 14.8 lbs
//       // 1.36f); // motor count - determined somewhat from data
//       // 2.5f); // motor count - determined somewhat from data
//       3.0f); // motor count - determined somewhat from data
//
//     LinearConstraints linear_constraints(
//       70_inps, // max vel - for testing
//       // 20.0_inps2, // max accel - for testing
//       10000.0_inps2, // max accel - for testing
//       // 150_inps2 // max decel - for testing also
//       200_inps2 // max decel - for testing also
//     );
//     //
//     // // TODO: what is the difference between angular accel/decel?
//     // AngularConstraints
//     // angular_constraints(2.0_radps, 1.3_radps2, 1.3_radps2);
//     AngularConstraints angular_constraints(2.0_radps,
//                                            // 1.3_radps2,
//                                            // 1.3_radps2
//
//                                            2.0_radps2,
//                                            2.0_radps2);
//     //
//     Constraints constraints(robot_constraints,
//                             linear_constraints,
//                             angular_constraints);
//     //
//     bool debug = true;
//     //
//     std::shared_ptr<Trajectory> test_trajectory(
//       new Trajectory(spline_ptr,
//                      constraints,
//                      {},
//                      {},
//                      // some initial velocity for it to move?
//                      // TODO: could there be a place on the curve that also
//                      has
//                      // a velof zero? if so this would also have the same
//                      issue? 0_inps, 0_inps, 0.1_in, debug));
//
//     trajectoryDebugPrint(test_trajectory.get());
//
//     // print out final trajectory and debug info
//
//     // drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
//
//     std::cout << "running path!" << std::endl;
//     // use path follow to follow the path
//     lyfast::PathFollow(controllers, velocity_chassis, test_trajectory)
//         .drive_toleranceDuration(100_sec)
//         .drive_largeToleranceDuration(100_sec)
//         .lookahead(20_msec + input_delay)
//         .reverse()
//         // .parameterization(blazing::lyfast::time_based)
//         .timeout(5_sec) |
//       run;
// }

void long_goal_to_match_test() {
    using namespace blazing::lyfast;
    using namespace blazing::lyfast::geometry;
    using namespace blazing::lyfast::mp;
    //
    // auto spline_ptr = skills_paths::urm_TO_urls2;
    auto spline_ptr = skills_paths::urls_TO_urm;
    // bool reversed = true;
    bool reversed = false;

    auto start_position = spline_ptr->getFirstEndpoint();
    auto start_angle = spline_ptr->df(0).getAngle();
    arc_pose_tracker.setPose({ start_position, start_angle });

    RobotConstraints robot_constraints(
      10.5_in, // track with
      // 0.05, // friction coeff - should tune?
      1.00, // friction coeff - should tune?
      3.25_in, // wheel diameter
      389_rpm, // max ang vel - determined somewhat from data
      6.7_kg, // about 14.8 lbs
      // 1.36f); // motor count - determined somewhat from data
      // 2.5f); // motor count - determined somewhat from data
      2.0f); // motor count - determined somewhat from data

    LinearConstraints linear_constraints(
      70_inps, // max vel - for testing
      // 20.0_inps2, // max accel - for testing
      10000.0_inps2, // max accel - for testing
      // 150_inps2 // max decel - for testing also
      150_inps2 // max decel - for testing also
    );
    //
    AngularConstraints angular_constraints(2.0_radps, 2.0_radps2, 2.0_radps2);
    //
    Constraints constraints(robot_constraints,
                            linear_constraints,
                            angular_constraints);
    //
    bool debug = true;
    //
    std::shared_ptr<Trajectory> test_trajectory(new Trajectory(
      spline_ptr,
      constraints,
      {},
      {
        // RangeConstraint { 0.7f, 1.0f, 20_inps, std::nullopt, std::nullopt }

      },
      2_inps,
      0_inps,
      0.1_in,
      debug));

    trajectoryDebugPrint(test_trajectory.get());

    // print out final trajectory and debug info

    // drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);

    std::cout << "running path!" << std::endl;
    // use path follow to follow the path
    lyfast::PathFollow(controllers, velocity_chassis, test_trajectory)
        .drive_toleranceDuration(100_sec)
        .drive_largeToleranceDuration(100_sec)
        .lookahead(20_msec + input_delay)
        .setReverse(reversed)
        // .parameterization(blazing::lyfast::time_based)
        .timeout(5_sec) |
      run;
}

AngularVelocity calculateMotorVel(int port, AngularVelocity max_vel) {
    static uint32_t previousInternalMotorClock[22];
    static int32_t oldMotorTicks[22];
    // oldMotorTicks[port] =
    //   test_motor.get_raw_position(&previousInternalMotorClock[port]);

    uint32_t currentInternalMotorClock;
    int32_t currentMotorTicks =
      pros::c::motor_get_raw_position(port, &currentInternalMotorClock);

    double dT = 5.0 * std::round((currentInternalMotorClock -
                                  previousInternalMotorClock[port]) /
                                 5.0);
    double dN = currentMotorTicks - oldMotorTicks[port];
    previousInternalMotorClock[port] = currentInternalMotorClock;
    oldMotorTicks[port] = currentMotorTicks;

    auto gearing_factor = (200_rpm / max_vel);

    // 900 ticks / revolution for 200 rpm cart
    // ticks decrease as final rpm increases
    Divided<Number, Angle> conversion = (900.0 / rot) * gearing_factor;

    Angle angular_position_delta = dN / conversion;
    AngularVelocity estimated_angular_velocity =
      angular_position_delta / from_msec(dT);

    return estimated_angular_velocity;
}

void logDrivetrainInformation(FDifferentialSpeeds curr_target_vels,
                              Voltage left_commanded_voltage,
                              Voltage right_commanded_voltage) {
    if (!drivetrain_tick_logging) return;

    left_tick_velocity.push_back(std::array<FLinearVelocity, 3> {
      toLinear(calculateMotorVel(left_back, final_rpm), wheel_diameter),
      toLinear(calculateMotorVel(left_middle, final_rpm), wheel_diameter),
      toLinear(calculateMotorVel(left_front, final_rpm), wheel_diameter) });

    right_tick_velocity.push_back(std::array<FLinearVelocity, 3> {
      toLinear(calculateMotorVel(right_back, final_rpm), wheel_diameter),
      toLinear(calculateMotorVel(right_middle, final_rpm), wheel_diameter),
      toLinear(calculateMotorVel(right_front, final_rpm), wheel_diameter) });

    Voltage left_filter_voltage = left_ema_filter.getInput();
    AngularVelocity left_filter_velocity = left_ema_filter.getPredictedState();

    Voltage right_filter_voltage = right_ema_filter.getInput();
    AngularVelocity right_filter_velocity =
      right_ema_filter.getPredictedState();

    left_filtered_data.emplace_back(
      toLinear(left_filter_velocity, wheel_diameter),
      left_filter_voltage);
    right_filtered_data.emplace_back(
      toLinear(right_filter_velocity, wheel_diameter),
      right_filter_voltage);
    target_velocities.push_back(curr_target_vels);

    left_target_voltage.push_back(left_commanded_voltage);
    right_target_voltage.push_back(right_commanded_voltage);
    //
    // AngularVelocity raw_vel = test_motor.get_actual_velocity() * rpm;

    // Torque torque = test_motor.get_torque() * Nm; // calculated
    // Current current = test_motor.get_current_draw() * mamp;
    // Power power = test_motor.get_power() * watt;

    // raw_data.emplace_back(raw_vel, commanded_voltage);
    // filtered_data.emplace_back(filter_velocity, filter_voltage);
    // extra_data.emplace_back(torque, current);
    // power_data.emplace_back(power);
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

    std::cout << "starting task: " << pros::micros() << std::endl;

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

    std::cout << "timed correctly: " << pros::micros() << '\n';

    while (1) {
        {
            frameCount++;

            // do stuff here
            if (frameCount % 5 == 0) {
                uint32_t curr_time = pros::millis();
                // Angle last_angle = arc_pose_tracker.getAngle();
                arc_pose_tracker.update();
                // Angle angle = arc_pose_tracker.getAngle();

                LinearVelocity forwards_velocity =
                  arc_pose_tracker.getLocalVelocityVector().x;

                // not using forwards velocity since its offset is not
                // guaranteed to be zero
                // LinearVelocity forwards_velocity =
                //   toLinear(from_degps(forwards_odom_rotation.get_velocity()),
                //            1.991_in);

                AngularVelocity angular_velocity =
                  -from_degps(imu.get_gyro_rate().z);

                LinearVelocity left_vel =
                  forwards_velocity - angular_velocity * 5.25_in / rad;
                LinearVelocity right_vel =
                  forwards_velocity + angular_velocity * 5.25_in / rad;

                LeftRightSpeeds measurement { left_vel, right_vel };

                // update plant
                drivetrain_plant.setMeasurement(measurement);
                drivetrain_plant.updateToTimestamp(curr_time);
                auto curr_drivetrain_voltages =
                  drivetrain_plant.getCommandedVoltages();

                left_motors.move_voltage(
                  12 * to_mvolt(curr_drivetrain_voltages.left_voltage));
                right_motors.move_voltage(
                  12 * to_mvolt(curr_drivetrain_voltages.right_voltage));
            }

            pros::Task::delay_until(&systemTime, 2);
        }
    }
}

void startTimeCriticalTask() {
    static bool daemonStarted = false;
    if (!daemonStarted) {
        pros::Task managerTask(timeCriticalTask,
                               15, // very high priority
                               TASK_STACK_DEPTH_DEFAULT,
                               "time critical task");
        daemonStarted = true;
    }
}

void initialize() {
    pros::lcd::initialize();

    imu.reset(true);

    startTimeCriticalTask();

    async.init();

    forwards_odom_rotation.set_data_rate(5);
    forwards_odom_rotation.set_data_rate(5);
    imu.set_data_rate(5);

    mb.setTurnToModifier([](auto turnTo) {
        std::cout << "modifying" << std::endl;
        std::ignore =
          turnTo
            ->velocity_based(true)
            // speecifically uses turn heading pid instead of drive pid
            .withAngularVelocityFeedbackController(turn_heading_vel_pid)
            // .withVelocityFeedforwardController(turn_vel_controller)
            .timeout(3_sec);

        std::cout << "did stuff prob" << std::endl;
    });

    mb.setArcModifier([](auto arc) -> auto {
        std::ignore =
          arc
            ->velocity_based(true)
            // speecifically uses turn heading pid instead of drive pid
            .withAngularVelocityFeedbackController(turn_heading_vel_pid)
            // .withVelocityFeedforwardController(turn_vel_controller)
            .timeout(3_sec);
    });

    //
    mb.setDistanceAtHeadingModifier([](auto distanceAtHeading) {
        std::ignore = distanceAtHeading->velocity_based(true).timeout(5_sec);
    });

    mb.setMoveToModifier([](auto moveTo) {
        std::ignore = moveTo->velocity_based(true)
                        .customAngularLinearFunc(angular_linear_func)
                        // .k_lat(0.0 * rad / m)
                        .timeout(3_sec);
    });

    mb.setBoomerangModifier([](auto boomerang) {
        std::ignore = boomerang->velocity_based(true)
                        .customAngularLinearFunc(angular_linear_func)
                        .k_lat(0.0 * rad / m, true)
                        .timeout(5_sec);
    });
}

// void moveToTest() {
//     arc_pose_tracker.setPose(units::Pose { 0_in, 0_in, 0_stDeg });
//     mb.moveTo(24_in, 0_in) | run;
// }

units::Pose RobotGetPose() {
    return { arc_pose_tracker.getPosition(), arc_pose_tracker.getAngle() };
}

void RobotSetPose(units::Pose pose) {
    arc_pose_tracker.setPose(pose);
}

void RobotSetPose(float x, float y, float theta) {
    RobotSetPose(units::Pose { x * in, y * in, theta * deg });
}

void drive_vel_pid_tuning() {
    Length target_distance = 24_in;
    Length target_distance_delta = 8_in;

    // Length target_lateral_distance = 2_in;
    Length target_lateral_distance = 24_in;

    double curr_kp =
      linear_angular_vel_pid.get_kp() / linear_angular_vel_pid.UKP;
    double curr_ki =
      linear_angular_vel_pid.get_ki() / linear_angular_vel_pid.UKI;
    double curr_kd =
      linear_angular_vel_pid.get_kd() / linear_angular_vel_pid.UKD;

    LinearAcceleration curr_accel_slew = 170_inps2;
    LinearAcceleration curr_max_accel = linear_mp_feedback.getMaxAccel();

    // Number curr_k_lat = 0.0;

    double kp_delta = 0.1;
    // double ki_delta = 0.01;
    double kd_delta = 0.1;

    LinearAcceleration slew_delta = 5_inps2;
    LinearAcceleration accel_delta = 5_inps2;

    bool config_swapped = false;
    bool reversed = false;

    units::V2Position start_position = { 0_in, 0_in };

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    std::cout << std::fixed << std::setprecision(3);

    while (true) {
        units::V2Position target_position =
          start_position +
          units::V2Position { target_distance, target_lateral_distance };

        drivetrain.setBrakeMode(pros::MotorBrake::hold);
        if (!reversed)
            RobotSetPose({ start_position, 0_stDeg });
        else
            RobotSetPose({ start_position, 180_stDeg });

        std::cout << "start is " << RobotGetPose().x.convert(in) << " "
                  << RobotGetPose().y.convert(in) << std::endl;

        auto start_time = from_msec(pros::millis());

        if (reversed) {
            // RobotSetPose(2 * target_distance.convert(in), 0, 0);
            mb.moveTo(target_position)
                // .drive_vel_kp(curr_kp)
                // .drive_vel_ki(curr_ki)
                // .drive_vel_kd(curr_kd)
                .drive_vel_accelSlew(curr_accel_slew)
                .drive_vel_mp_setMaxAccel(curr_max_accel)

                // .lateral_vel_kp(curr_kp)
                // .lateral_vel_ki(curr_ki)
                // .lateral_vel_kd(curr_kd)

                .turn_vel_kp(curr_kp)
                .turn_vel_ki(curr_ki)
                .turn_vel_kd(curr_kd)
                .timeout(3.3_sec)

                //
                // .k_lat(0)
                .reverse()
              // .closeThreshold(7_in)
              | run;
        } else {
            // RobotSetPose(0, 0, 0);
            mb.moveTo(target_position)
                // .drive_vel_kp(curr_kp)
                // .drive_vel_ki(curr_ki)
                // .drive_vel_kd(curr_kd)
                .drive_vel_accelSlew(curr_accel_slew)
                .drive_vel_mp_setMaxAccel(curr_max_accel)

                // .turn_vel_kp(0)
                // .turn_vel_ki(0)
                // .turn_vel_kd(0)

                .turn_vel_kp(curr_kp)
                .turn_vel_ki(curr_ki)
                .turn_vel_kd(curr_kd)

                // .lateral_vel_kp(curr_kp)
                // .lateral_vel_ki(curr_ki)
                // .lateral_vel_kd(curr_kd)

                .timeout(3.3_sec)
              // .drive_errorTolerance(0_in)

              // .closeThreshold(7_in)
              | run;
        }

        controller.rumble(".");

        const auto end_time = from_msec(pros::millis());

        const auto time_difference = end_time - start_time;

        const auto curr_pose = RobotGetPose();
        const auto error_vec = target_position - curr_pose;

        std::cout << "final error: " << error_vec.magnitude().convert(in)
                  << ", x: " << error_vec.x.convert(in)
                  << ", y: " << error_vec.y.convert(in) << std::endl;

        auto local_error_vec = error_vec.rotatedBy(-curr_pose.orientation);

        std::cout << "final local error: "
                  << local_error_vec.magnitude().convert(in)
                  << ", forwards: " << local_error_vec.x.convert(in)
                  << ", sideways: " << local_error_vec.y.convert(in)
                  << std::endl;

        // clang-format off
        std::cout << "position: "
				  << curr_pose.x.convert(in) << " "
                  << curr_pose.y.convert(in) << " "
                  << curr_pose.orientation.convert(deg) << std::endl;
        // clang-format on

        std::cout << "took " << time_difference.convert(sec)
                  << " time to finish turn" << std::endl;

        while (
          !controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_A)) {
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_LEFT)) {
                target_distance -= target_distance_delta;
                std::cout << "decreased target to "
                          << target_distance.convert(in) << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_B)) {
                // move back
                if (reversed) {
                    mb.moveTo(start_position)
                        .timeout(2_sec)
                        .closeThreshold(7_in)
                      // .drive_vel_maxVel(20_inps)
                      | async;
                } else {
                    mb.moveTo(start_position)
                        .timeout(3_sec)
                        .closeThreshold(7_in)
                        // .drive_vel_maxVel(20_inps)
                        .reverse() |
                      async;
                }
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_X)) {

                // turns around
                // if (reversed) {
                //     mb.turnTo(0) | async;
                // } else {
                //     mb.turnTo(180) | async;
                // }
                //
                // reversed = !reversed;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_Y)) {
                config_swapped = !config_swapped;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                target_distance += target_distance_delta;
                std::cout << "increased target to "
                          << target_distance.convert(in) << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L2)) {
                curr_kp -= kp_delta;
                std::cout << "decreased kp to " << curr_kp << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L1)) {
                curr_kp += kp_delta;
                std::cout << "increased kp to " << curr_kp << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R2)) {
                curr_kd -= kd_delta;
                std::cout << "decreased kd to " << curr_kd << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R1)) {
                curr_kd += kd_delta;
                std::cout << "increased kd to " << curr_kd << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_UP)) {
                if (config_swapped) {
                    curr_max_accel += accel_delta;
                    std::cout << "increased max accel to "
                              << curr_max_accel.internal() << std::endl;
                } else {
                    curr_accel_slew += slew_delta;
                    std::cout << "increased slew to "
                              << curr_accel_slew.internal() << std::endl;
                }
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_DOWN)) {
                if (config_swapped) {
                    curr_max_accel -= accel_delta;
                    std::cout << "decreased max accel to "
                              << curr_max_accel.internal() << std::endl;
                } else {
                    curr_accel_slew -= slew_delta;
                    std::cout << "decreased slew to "
                              << curr_accel_slew.internal() << std::endl;
                }
            }
            // kp = 7
            // kd = 10.5

            // kp = 6.3
            // kd = 10.2
            pros::delay(10);
        }
    }
}

void turn_vel_pid_tuning() {
    double target_theta = 90;
    // by how much we can increase or decrease
    double target_theta_delta = 45;

    double curr_kp = turn_heading_vel_pid.get_kp() / turn_heading_vel_pid.UKP;
    double curr_ki = turn_heading_vel_pid.get_ki() / turn_heading_vel_pid.UKI;
    double curr_kd = turn_heading_vel_pid.get_kd() / turn_heading_vel_pid.UKD;

    double kp_delta = 0.10;
    double ki_delta = 0.01;
    double kd_delta = 0.20;

    pros::delay(2000);

    std::cout << std::fixed << std::setprecision(3);

    while (true) {
        // matchloader::down();

        RobotSetPose(0, 0, 0);
        auto start_time = from_msec(pros::millis());

        mb.turnTo(target_theta)
            // .turn_vel_maxVel(200_degps)
            // .direction(AngularDirection::RIGHT)
            // .radius(-10.5_in)
            .turn_vel_kp(curr_kp)
            .turn_vel_ki(curr_ki)
            .turn_vel_kd(curr_kd)
            .timeout(3.0_sec) |
          run;

        // mb_vel.moveTo(-48_in, 3_in)
        //     .reverse()
        //     .drive_vel_minVel(50_inps)
        //     // .drive_vel_mp_maxVel(100_inps)
        //     // .drive_vel_mp_setMaxAccel(300_inps2)
        //     .setChainTime(0_msec) |
        //   chain;

        // mb_vel.turnTo(target_theta)
        //     .turn_vel_kp(curr_kp)
        //     .turn_vel_ki(curr_ki)
        //     .turn_vel_kd(curr_kd)
        //     // .turn_vel_PIDmaxVel(30_degps)
        //     .reverse()
        //     // .turn_vel_maxVel(30_degps)
        //     .radius(-10.5_in / 2.0)
        //     .timeout(5_sec) |
        //   chain;
        // chain.wait();

        // left_motors.set_brake_mode(pros::MotorBrake::brake);
        //
        // pros::delay(1500);
        // drivetrain.moveTank(0_volt, 0_volt);

        // drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
        // RobotSetPose(-30, 57, 180);
        //
        // mb.moveTo(17_in, 54_in)
        //     .reverse()
        //     .drive_vel_minVel(50_inps)
        //     .setChainTime(0_sec) |
        //   chain;
        //
        // // mb.turnTo(21.8_in, 47_in)
        // mb.turnTo(180)
        //     .reverse()
        //     .direction(AngularDirection::RIGHT)
        //     .radius(-10.5_in / 2)
        //     .timeout(2.6_sec)
        //     .setChainTime(0_sec) |
        //   chain;
        // mb.turnTo(0).radius(-4_in).timeout(2.6_sec) | chain;
        //
        // chain.wait();

        // mb.turnTo(target_theta)
        //     // .turn_vel_maxVel(200_degps)
        //     .direction(AngularDirection::RIGHT)
        //     .radius(-10.5_in)
        //     // .turn_vel_kp(curr_kp)
        //     // .turn_vel_ki(curr_ki)
        //     // .turn_vel_kd(curr_kd)
        //     .timeout(2.6_sec) |
        //   run;

        // drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
        // RobotSetPose(48, 48, 0);
        // mb.turnTo(47, 47).timeout(3.0_sec) | run;

        controller.rumble(".");

        const auto end_time = from_msec(pros::millis());

        const auto time_difference = end_time - start_time;

        std::cout << "final error was "
                  << target_theta - RobotGetPose().orientation.convert(deg)
                  << std::endl;

        // clang-format off
        std::cout << "position: "
				  << RobotGetPose().x.convert(in) << " "
                  << RobotGetPose().y.convert(in) << " "
                  << RobotGetPose().orientation.convert(deg) << std::endl;
        // clang-format on

        std::cout << "took " << time_difference.convert(sec)
                  << " time to finish turn" << std::endl;

        while (
          !controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_A)) {
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_LEFT)) {
                target_theta -= target_theta_delta;
                std::cout << "decreased to " << target_theta << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                target_theta += target_theta_delta;
                std::cout << "increased to " << target_theta << std::endl;
            }

            if (controller.get_digital_new_release(controls::DOWN)) {
                curr_ki -= ki_delta;
                std::cout << "ki - to " << curr_ki << std::endl;
            }
            if (controller.get_digital_new_release(controls::UP)) {
                curr_ki += ki_delta;
                std::cout << "ki + to " << curr_ki << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L2)) {
                curr_kp -= kp_delta;
                std::cout << "decreased kp to " << curr_kp << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L1)) {
                curr_kp += kp_delta;
                std::cout << "increased kp to " << curr_kp << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R2)) {
                curr_kd -= kd_delta;
                std::cout << "decreased kd to " << curr_kd << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R1)) {
                curr_kd += kd_delta;
                std::cout << "increased kd to " << curr_kd << std::endl;
            }

            pros::delay(10);
        }
    }
}

// void odom_offset_tuning() {
//     sideways_odom_rotation.set_position(0);
//     forwards_odom_rotation.set_position(0);
//     pros::delay(10);
//
//     double last_sideways_rotation = sideways_odom_rotation.get_position();
//     double last_forwards_rotation = forwards_odom_rotation.get_position();
//     Angle last_angle = arc_pose_tracker.getAngle();
//
//     auto get_offset =
//       [](double distance_delta, Length wheel_diameter, Angle angle_delta) {
//           const double rotations = (distance_delta * deg / 100.0) / rot;
//
//           const Length measured = rotations * (wheel_diameter * M_PI);
//           return measured / to_stRad(angle_delta);
//       };
//
//     Time last_measurement_time = now();
//
//     while (true) {
//         velocity_drivetrain.moveArcade(0_inps, 3.0_radps);
//         // left_motors.move_voltage(-12000 * pct);
//         // right_motors.move_voltage(12000 * pct);
//
//         // units::V2Position deltas = { forwards_tracker.getDelta(),
//         //                              sideways_tracker.getDelta() };
//         // Angle delta_theta = imu_tracker.getDelta();
//         //
//         // units::V2Position offsets = deltas / to_stRad(delta_theta);
//
//         // gets offsets every 0.2 seconds
//         if (blazing::timeoutDone(0.2_sec, last_measurement_time)) {
//             last_measurement_time = now();
//             Angle angle_delta = arc_pose_tracker.getAngle() - last_angle;
//             last_angle = arc_pose_tracker.getAngle();
//
//             double curr_forwards_rotation =
//               forwards_odom_rotation.get_position();
//             double curr_sideways_rotation =
//               sideways_odom_rotation.get_position();
//
//             double forwards_delta =
//               curr_forwards_rotation - last_forwards_rotation;
//             double sideways_delta =
//               curr_sideways_rotation - last_sideways_rotation;
//
//             last_forwards_rotation = curr_forwards_rotation;
//             last_sideways_rotation = curr_sideways_rotation;
//
//             units::V2Position offsets = {
//                 get_offset(forwards_delta, 1.991_in, angle_delta),
//                 get_offset(sideways_delta, 1.991_in, angle_delta)
//             };
//
//             std::cout << offsets.x.convert(in) << " " <<
//             offsets.y.convert(in)
//                       << std::endl;
//         }
//
//         pros::delay(20);
//     }
// }

// }

void findImuOrientation() {
    pros::imu_orientation_e_t imu_orientation = imu.get_physical_orientation();

    if (imu_orientation == pros::E_IMU_X_DOWN)
        std::cout << "E_IMU_X_DOWN" << std::endl;
    if (imu_orientation == pros::E_IMU_Y_DOWN)
        std::cout << "E_IMU_Y_DOWN" << std::endl;
    if (imu_orientation == pros::E_IMU_Z_DOWN)
        std::cout << "E_IMU_Z_DOWN" << std::endl;
    if (imu_orientation == pros::E_IMU_X_UP)
        std::cout << "E_IMU_X_UP" << std::endl;
    if (imu_orientation == pros::E_IMU_Y_UP)
        std::cout << "E_IMU_Y_UP" << std::endl;
    if (imu_orientation == pros::E_IMU_Z_UP)
        std::cout << "E_IMU_Z_UP" << std::endl;
}

void opcontrol() {
    arc_pose_tracker.setPose({ -23.6_in, 0_in, 0_stDeg });
    pros::Task([] {
        while (true) {
            auto pos = arc_pose_tracker.getPosition();
            auto theta = arc_pose_tracker.getAngle();

            pros::lcd::print(0,
                             "%.2f %.2f %.2f",
                             pos.x.convert(in),
                             pos.y.convert(in),
                             theta.convert(deg));

            // pros::lcd::print(
            //   2,
            //   "%.2f   %.2f",
            //   (velocity_drivetrain.getDrivetrainVelocities().right_vel -
            //    velocity_drivetrain.getDrivetrainVelocities().left_vel) /
            //     track_width,
            //   -from_degps(imu.get_gyro_rate().z));

            FLinearVelocity rotation_velocity =
              toLinear(from_degps(forwards_odom_rotation.get_velocity()),
                       1.991_in);

            pros::lcd::print(1,
                             "%.5f",
                             ((drivetrain.getDrivetrainVelocities().right_vel +
                               drivetrain.getDrivetrainVelocities().left_vel) /
                              2)
                               .internal());

            // pros::lcd::print(2, "%.5f", rotation_velocity.internal());
            pros::lcd::print(2, "%.5f", rotation_velocity.internal());
            pros::delay(100);
        }
    });
    // findImuOrientation();
    // path_follow_test();

    // long_goal_to_match_test();

    // moveToTest();
    drive_vel_pid_tuning();
    // turn_vel_pid_tuning();
}
