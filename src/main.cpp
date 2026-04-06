#include "main.h"
#include "blazing/api.hpp"
#include "blazing/latex_utils.hpp"
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
#include <tuple>
#include <utility>
#include <variant>

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
// DifferentialDrivetrain
//   drivetrain(&left_motors, &right_motors, wheel_diameter, final_rpm);

lyfast::DifferentialVelocityControllerParams vel_controller_params {
	.linear = {
		// TODO: recalc angular?
		// .left_Kv = 0.46 * volt / mps,
		.left_Kv = 0.46 * volt / mps,
		.left_Ka = 0.09 * volt / mps2,
		// .left_low_target_Kv = 0.4 * volt / mps,
		// .left_low_target_Ka = 0.0 * volt / mps2,
		.left_Ks = 0.08 * volt,

		// .right_Kv = 0.49 * volt / mps,
		.right_Kv = 0.47 * volt / mps,
		.right_Ka = 0.09 * volt / mps2,
		// .right_low_target_Kv = 0.42 * volt / mps,
		// .right_low_target_Ka = 0.0 * volt / mps2,
		.right_Ks = 0.08 * volt,

		.Ka_delta_time = 20_msec,
		// .low_target_threshold = 2_inps
		// .low_target_threshold = 4_inps
		.low_target_threshold = -1_inps
	},
	.angular = {
		// .left_Kv = 0.87 * volt / mps,
		// .left_Kv = 0.7 * volt / mps,
		.left_Kv = 0.90 * volt / mps,
		.left_Ka = 0.08 * volt / mps2,
		// .left_Ka = 0.10 * volt / mps2,
		// .left_Ka = 0.08 * volt / mps2,
		.left_low_target_Kv = 0.4 * volt / mps,
		.left_low_target_Ka = 0.03 * volt / mps2,
		.left_Ks = 0.08 * volt,

		// .right_Kv = 0.87 * volt / mps,
		.right_Kv = 0.90 * volt / mps,
		// .right_Ka = 0.10 * volt / mps2,
		// .right_Ka = 0.08 * volt / mps2,
		.right_Ka = 0.08 * volt / mps2,
		.right_low_target_Kv = 0.4 * volt / mps,
		.right_low_target_Ka = 0.03 * volt / mps2,
		.right_Ks = 0.08 * volt,


		.Ka_delta_time = 20_msec,
		// .low_target_threshold = 2_inps
		.low_target_threshold = 7_inps
	},
	.linear_pid = {
		// .left_Kp = 1.5 * volt / mps,
		// .left_Kp_close = 0.0 * volt / mps,
		// .left_Kp_low = 0.0 * volt / mps,
		// .left_low_threshold = 5_inps,
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
		// .right_low_threshold = 5_inps,
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
ForwardsTracker forwards_tracker(&forwards_odom_rotation, 0.04_in, 1.991_in);
SidewaysTracker sideways_tracker(&sideways_odom_rotation, -2.1_in, 1.991_in);

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

// Chassis chassis(drivetrain, arc_pose_tracker, tolerances);
Chassis velocity_chassis(&velocity_drivetrain, &arc_pose_tracker, tolerances);

RunExecutor run;
AsyncExecutor async;

FLength track_radius = track_width * 0.5;

FLinearVelocity max_velocity = 76_Finps;
// w = v / r
FAngularVelocity max_angular_velocity = (max_velocity / track_radius) * Frad;

std::array<float, 3> Q { (40_in).internal(),
                         // (6_in).internal(),
                         (5_in).internal(),
                         // (1_in).internal(),
                         // (5_stDeg).internal() };
                         (180_stDeg).internal() };

// [x, theta]
std::array<float, 2> simple_Q { (30_in).internal(), (100_stDeg).internal() };

std::array<float, 2> R { // max velocity
                         max_velocity.internal(),
                         // max angular velocity
                         max_angular_velocity.internal()
};

Time input_delay = 40_msec;
LinearVelocity lqr_minimum_velocity = 1.0_inps;

blazing::lyfast::state_space::LTVUnicycleController
  lqr_controller(Q, simple_Q, R, 0_msec, lqr_minimum_velocity);

blazing::lyfast::RamsetteController ramsete_controller(1, 0.5);
blazing::lyfast::NoPathFeedbackController no_feedback_controller;

// lyfast::PathPoseFeedbackController<decltype(lqr_controller)>
// path_pose_feedback_controller(lqr_controller);
// lyfast::PathPoseFeedbackController<decltype(ramsete_controller)>
  // path_pose_feedback_controller(ramsete_controller);
lyfast::PathPoseFeedbackController<decltype(no_feedback_controller)>
path_pose_feedback_controller(no_feedback_controller);

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

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void genericTuner(
  const std::string& type,
  Time delta_time,
  std::function<lyfast::sysid::DifferentialData()> gatherData,
  std::function<void(const lyfast::sysid::DifferentialData&)> processData) {
    lyfast::sysid::DifferentialData data;

    while (true) {
        velocity_drivetrain.moveTank(0_volt, 0_volt);

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            data = gatherData();

            processData(data);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "data: " << std::endl;
            lyfast::sysid::DifferentialUtils::printData(data, delta_time);
            printDrivetrainData();
        }
        pros::delay(10);
    }
}

void kv_ks_tuner(const std::string& type,
                 const std::vector<lyfast::sysid::DifferentialVoltageCommand>&
                   voltage_commands,
                 bool use_measured_voltage = false,
                 Time steady_state_time = 150_msec,
                 Time delta_time = 10_msec) {
    using namespace lyfast::sysid;
    // function params outlive the genericTuner function, so capturing
    // them by reference should fine
    genericTuner(
      type,
      delta_time,
      [&] -> DifferentialData {
          return DifferentialUtils::generate_kv_ks_data(voltage_commands,
                                                        velocity_drivetrain,
                                                        delta_time,
                                                        steady_state_time,
                                                        use_measured_voltage);
      },
      [](const DifferentialData& data) {
          DifferentialUtils::calculate_kv_ks(data);
      });
}

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void raw_ka_tuner(const std::string& type,
                  const std::vector<lyfast::sysid::DifferentialVoltageCommand>&
                    voltage_commands,
                  lyfast::KvUnits<LinearVelocity> left_Kv,
                  lyfast::KsUnits left_Ks,
                  lyfast::KvUnits<LinearVelocity> right_Kv,
                  lyfast::KsUnits right_Ks,
                  bool use_measured_voltage = true,
                  Time delta_time = 10_msec) {
    using namespace lyfast::sysid;
    // function params outlive the genericTuner function, so capturing
    // them by reference should fine
    genericTuner(
      type,
      delta_time,
      [&] {
          return DifferentialUtils::generateData(voltage_commands,
                                                 velocity_drivetrain,
                                                 delta_time,
                                                 use_measured_voltage);
      },
      [&](const DifferentialData& data) {
          DifferentialUtils::calculate_ka(data,
                                          left_Kv,
                                          left_Ks,
                                          right_Kv,
                                          right_Ks,
                                          delta_time);
      });
}

void create_accel_data(
  const lyfast::sysid::DifferentialVoltageCommand& voltage_command,
  const std::string& type,
  Time delta_time = 10_msec) {
    using namespace lyfast::sysid;
    // function params outlive the genericTuner function, so capturing
    // them by reference should fine
    genericTuner(
      type,
      delta_time,
      [&] {
          return DifferentialUtils::generateData({ voltage_command },
                                                 velocity_drivetrain,
                                                 delta_time);
      },
      [](const DifferentialData& data) {});
}

void ka_kp_ki_tuner(
  const std::string& type,
  const lyfast::sysid::DifferentialVoltageCommand& voltage_command,
  double lambda_factor,
  bool use_measured_voltage = false,
  Time delta_time = 10_msec) {
    using namespace lyfast::sysid;
    // function params outlive the genericTuner function, so capturing
    // them by reference should fine
    genericTuner(
      type,
      delta_time,
      [&] {
          return DifferentialUtils::generateData({ voltage_command },
                                                 velocity_drivetrain,
                                                 delta_time,
                                                 use_measured_voltage);
      },
      [&](const DifferentialData& data) {
          DifferentialUtils::calculate_ka_kp_ki_fopdt(data,
                                                      delta_time,
                                                      lambda_factor);
      });
}

void linear_ka_kp_ki_tuner(Voltage u_step = 0.5_volt,
                           double lambda_factor = 0.6,
                           Time accel_time = 2_sec,
                           bool use_measured_voltage = false,
                           Time delta_time = 10_msec) {
    ka_kp_ki_tuner("LINEAR",
                   { u_step, u_step, accel_time },
                   lambda_factor,
                   use_measured_voltage,
                   delta_time);
}

void angular_ka_kp_ki_tuner(Voltage u_step = 0.5_volt,
                            double lambda_factor = 0.6,
                            Time accel_time = 2_sec,
                            bool use_measured_voltage = false,
                            Time delta_time = 10_msec) {
    ka_kp_ki_tuner("ANGULAR",
                   { u_step, -u_step, accel_time },
                   lambda_factor,
                   use_measured_voltage,
                   delta_time);
}

void linear_kv_ks_tuner(bool use_measured_voltage = false,
                        Time steady_state_time = 100_msec,
                        Time delta_time = 10_msec) {
    kv_ks_tuner("LINEAR",
                std::vector<lyfast::sysid::DifferentialVoltageCommand> {
                  // linear movements
                  { -0.1_volt, -0.1_volt, 600_msec },
                  { 0.0_volt, 0.0_volt, 500_msec, false },
                  { 0.2_volt, 0.2_volt, 1300_msec },
                  { 0.0_volt, 0.0_volt, 500_msec, false },
                  { -0.3_volt, -0.3_volt, 1300_msec },
                  { 0.0_volt, 0.0_volt, 500_msec, false },
                  { 0.4_volt, 0.4_volt, 1300_msec },
                  { 0.0_volt, 0.0_volt, 500_msec, false },
                  { -0.5_volt, -0.5_volt, 1300_msec },
                  { 0.0_volt, 0.0_volt, 500_msec, false },
                  { 0.6_volt, 0.6_volt, 1300_msec },
                  { 0.0_volt, 0.0_volt, 500_msec, false },
                  { -0.7_volt, -0.7_volt, 1300_msec },
                  { 0.0_volt, 0.0_volt, 500_msec, false },
    },
                use_measured_voltage,
                steady_state_time,
                delta_time);
}

void angular_kv_ks_tuner(bool use_measured_voltage = false,
                         Time steady_state_time = 100_msec,
                         Time delta_time = 10_msec) {
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
                use_measured_voltage,
                steady_state_time,
                delta_time);
}

void linear_raw_ka_tuner(lyfast::KvUnits<LinearVelocity> left_Kv,
                         lyfast::KsUnits left_Ks,
                         lyfast::KvUnits<LinearVelocity> right_Kv,
                         lyfast::KsUnits right_Ks,
                         bool use_measured_voltage = false,
                         Time delta_time = 10_msec) {
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
                 right_Ks,
                 use_measured_voltage,
                 delta_time);
}

void angular_raw_ka_tuner(lyfast::KvUnits<LinearVelocity> left_Kv,
                          lyfast::KsUnits left_Ks,
                          lyfast::KvUnits<LinearVelocity> right_Kv,
                          lyfast::KsUnits right_Ks,
                          bool use_measured_voltage = false,
                          Time delta_time = 10_msec) {
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
    raw_ka_tuner("ANGULAR",
                 mixed_voltage_commands,
                 left_Kv,
                 left_Ks,
                 right_Kv,
                 right_Ks,
                 use_measured_voltage,
                 delta_time);
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

void path_follow_test() {

    using namespace blazing::lyfast;
    using namespace blazing::lyfast::geometry;
    using namespace blazing::lyfast::mp;
    //
    std::shared_ptr<Line> line(new Line({ -23.6_in, 0_in }, { 0_in, 0_in }));
    std::shared_ptr<CubicBezier> bezier(new CubicBezier({ 0_in, 0_in },
                                                        { 23.6_in, 0_in },
                                                        { 23.6_in, 23.6_in },
                                                        { 47.2_in, 23.6_in }));

    // std::shared_ptr<geometry::Spline> spline_ptr { new Spline(
    //   { line, bezier }) };

    // auto spline_ptr = skills_paths::ulm_TO_url1;

    std::shared_ptr<geometry::Spline> spline_ptr { new Spline(
      { skills_paths::ulm_TO_url1, skills_paths::url1_TO_End_Control }) };

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
      3.0f); // motor count - determined somewhat from data

    LinearConstraints linear_constraints(
      70_inps, // max vel - for testing
      // 20.0_inps2, // max accel - for testing
      10000.0_inps2, // max accel - for testing
      // 150_inps2 // max decel - for testing also
      200_inps2 // max decel - for testing also
    );
    //
    // // TODO: what is the difference between angular accel/decel?
    // AngularConstraints
    // angular_constraints(2.0_radps, 1.3_radps2, 1.3_radps2);
    AngularConstraints angular_constraints(1.0_radps, 1.3_radps2, 1.3_radps2);
    //
    Constraints constraints(robot_constraints,
                            linear_constraints,
                            angular_constraints);
    //
    bool debug = true;
    //
    std::shared_ptr<Trajectory> test_trajectory(
      new Trajectory(spline_ptr,
                     constraints,
                     {},
                     {},
                     // some initial velocity for it to move?
                     // TODO: could there be a place on the curve that also has
                     // a velof zero? if so this would also have the same issue?
                     0_inps,
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
        // .parameterization(blazing::lyfast::time_based)
        .timeout(5_sec) |
      run;
}

pros::MotorGroup test_motor({ 13 }, pros::MotorGears::green);

// test motor kv ks and ka
auto Kv = 0.039394 * volt / radps;
auto Ks = 0.027303 * volt;
auto Ka = 0.001357 * volt / radps2;

auto Kp = 0.05 * volt / radps;
auto Ki = 0.02 * volt / rad;

auto motor_voltage_Kv = 0.039394 * volt / radps;
auto motor_voltage_Ks = 0.027303 * volt;
auto motor_voltage_Ka = 0.001357 * volt / radps2;

lyfast::EMAVelocityFilter::Constants test_motor_ema_filter_constants {
    .final_gearing_rpm = 200_rpm,
    .Koffset = 0.1,
    .KalphaFactor = 0.1,
};
lyfast::EMAVelocityFilter test_motor_filter { &test_motor,
                                              test_motor_ema_filter_constants };

lyfast::FeedforwardVelocityController<AngularVelocity> feedforward {
    lyfast::FeedforwardVelocityControllerParams<AngularVelocity> {
                                                                  .Kv = motor_voltage_Kv,
                                                                  .Ka = motor_voltage_Ka,
                                                                  .Ks = motor_voltage_Ks,
                                                                  .Ka_delta_time = 10_msec,
                                                                  .low_target_threshold = 0_radps }
};
lyfast::PIDVelocityController<AngularVelocity> feedback {
    lyfast::PIDVelocityControllerParams<AngularVelocity> {
                                                          .Kp = Kp,
                                                          .Ki = Ki,
                                                          .max_output = 1.0 * volt,
                                                          .tbh_factor = 1.0,
                                                          }
};
lyfast::SimpleVelocityController<AngularVelocity> controller { feedforward,
                                                               feedback };
lyfast::AngularMotorGroupVelocityPlant test_plant(&test_motor_filter,
                                                  controller);

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

    Voltage filter_voltage = test_motor_filter.getInput();
    AngularVelocity filter_velocity = test_motor_filter.getPredictedState();

    AngularVelocity raw_vel = test_motor.get_actual_velocity() * rpm;

    Torque torque = test_motor.get_torque() * Nm; // calculated
    Current current = test_motor.get_current_draw() * mamp;
    Power power = test_motor.get_power() * watt;

    raw_data.emplace_back(raw_vel, commanded_voltage);
    filtered_data.emplace_back(filter_velocity, filter_voltage);
    extra_data.emplace_back(torque, current);
    power_data.emplace_back(power);
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

                // predict the filter
                // test_motor_filter.predictToTimestamp(curr_time);
                // test_motor_filter.correct();
                // // update plant
                // test_plant.updateToTimestamp(curr_time);
                //
                // Voltage test_plant_voltage =
                // test_plant.getCommandedVoltage();
                //
                // // actuate motor
                // test_motor.move_voltage(12 *
                // to_mvolt(test_plant_voltage));
                // logInformation(test_plant_voltage);

                // update drivetrain filters
                // std::cout << "left:";
                left_ema_filter.predictToTimestamp(curr_time);
                left_ema_filter.correct();
                // std::cout << "\n";

                // std::cout << "right: ";
                right_ema_filter.predictToTimestamp(curr_time);
                right_ema_filter.correct();
                // std::cout << "\n";

                Angle last_angle = arc_pose_tracker.getAngle();
                arc_pose_tracker.update();
                Angle angle = arc_pose_tracker.getAngle();

                LinearVelocity forwards_velocity =
                  arc_pose_tracker.getLocalVelocityVector().x;
                AngularVelocity angular_velocity =
                  (angle - last_angle) / 10_msec;

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

                // DifferentialSpeeds speeds =
                //   std::holds_alternative<DifferentialSpeeds>(
                //     drivetrain_plant.getTarget()) ?
                //     std::get<DifferentialSpeeds>(drivetrain_plant.getTarget())
                //     : DifferentialSpeeds { 0_inps, 0_radps };

                // std::cout
                //   <<
                //   velocity_drivetrain.getDrivetrainVelocities().left_vel
                //   << " "
                //   <<
                //   velocity_drivetrain.getDrivetrainVelocities().right_vel
                //   << "\n";

                // actuate motors with desired voltages
                // int discretized_left_voltage = round(
                //   curr_drivetrain_voltages.left_voltage.internal() * 100.0);
                // int discretized_right_voltage = round(
                //   curr_drivetrain_voltages.right_voltage.internal() * 100.0);
                //
                // left_motors.move_voltage((12000 / 100) *
                //                          discretized_left_voltage);
                // right_motors.move_voltage((12000 / 100) *
                //                           discretized_right_voltage);

                left_motors.move_voltage(
                  12 * to_mvolt(curr_drivetrain_voltages.left_voltage));
                right_motors.move_voltage(
                  12 * to_mvolt(curr_drivetrain_voltages.right_voltage));

                // logDrivetrainInformation(
                //   FDifferentialSpeeds { speeds.linear_velocity,
                //                         speeds.angular_velocity },
                //   volt * discretized_left_voltage / 100.0,
                //   volt * discretized_right_voltage / 100.0);
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
    // pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
    pros::lcd::initialize();

    imu.reset(true);

    // pros::Task([&] {
    //     while (true) {
    //         arc_pose_tracker.update();
    //         pros::delay(10);
    //     }
    // });

    startTimeCriticalTask();

    forwards_odom_rotation.set_data_rate(5);
    forwards_odom_rotation.set_data_rate(5);
    imu.set_data_rate(5);
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

    // AngularVelocity final_rpm = 200_rpm;

    auto vel_func = [&]() -> AngularVelocity {
        // return get_group_velocity(&test_motor, final_rpm);
        // can use filtered data for kv/ks
        return test_motor_filter.getPredictedState();
    };
    auto voltage_func = [&](Voltage commanded_voltage) -> Voltage {
        // using either will return different values?
        // return commanded_voltage;
        return getGroupVoltage(&test_motor);
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
        return getGroupVelocity(&test_motor, final_rpm);
    };
    auto voltage_func = [&](Voltage commanded_voltage) -> Voltage {
        // using either will return different values?
        // return commanded_voltage;
        return getGroupVoltage(&test_motor);
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

    std::vector<std::pair<AngularVelocity, Time>> test_commands = {
        { 0.1 * 200_rpm,  400_msec  },
        { 0.3 * 200_rpm,  500_msec  },
        { 0.4 * 200_rpm,  100_msec  },
        { 0.2 * 200_rpm,  100_msec  },
        { -0.5 * 200_rpm, 600_msec  },
        { -1.0 * 200_rpm, 600_msec  },
        { 1.0 * 200_rpm,  1000_msec },
    };

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

void odom_offset_tuning() {
    sideways_odom_rotation.set_position(0);
    forwards_odom_rotation.set_position(0);
    pros::delay(10);

    double last_sideways_rotation = sideways_odom_rotation.get_position();
    double last_forwards_rotation = forwards_odom_rotation.get_position();
    Angle last_angle = arc_pose_tracker.getAngle();

    auto get_offset =
      [](double distance_delta, Length wheel_diameter, Angle angle_delta) {
          const double rotations = (distance_delta * deg / 100.0) / rot;

          const Length measured = rotations * (wheel_diameter * M_PI);
          return measured / to_stRad(angle_delta);
      };

    Time last_measurement_time = now();

    while (true) {
        velocity_drivetrain.moveArcade(0_inps, 3.0_radps);
        // left_motors.move_voltage(-12000 * pct);
        // right_motors.move_voltage(12000 * pct);

        // units::V2Position deltas = { forwards_tracker.getDelta(),
        //                              sideways_tracker.getDelta() };
        // Angle delta_theta = imu_tracker.getDelta();
        //
        // units::V2Position offsets = deltas / to_stRad(delta_theta);

        // gets offsets every 0.2 seconds
        if (blazing::timeoutDone(0.2_sec, last_measurement_time)) {
            last_measurement_time = now();
            Angle angle_delta = arc_pose_tracker.getAngle() - last_angle;
            last_angle = arc_pose_tracker.getAngle();

            double curr_forwards_rotation =
              forwards_odom_rotation.get_position();
            double curr_sideways_rotation =
              sideways_odom_rotation.get_position();

            double forwards_delta =
              curr_forwards_rotation - last_forwards_rotation;
            double sideways_delta =
              curr_sideways_rotation - last_sideways_rotation;

            last_forwards_rotation = curr_forwards_rotation;
            last_sideways_rotation = curr_sideways_rotation;

            units::V2Position offsets = {
                get_offset(forwards_delta, 1.991_in, angle_delta),
                get_offset(sideways_delta, 1.991_in, angle_delta)
            };

            std::cout << offsets.x.convert(in) << " " << offsets.y.convert(in)
                      << std::endl;
        }

        pros::delay(20);
    }
}

// }
void opcontrol() {
    // pros::delay(2000);
    // motorPlantTest();
    // linear_kv_ks_tuner(true);
    // linear_raw_ka_tuner(vel_controller_params.linear.left_Kv,
    //                     vel_controller_params.linear.left_Ks,
    //                     vel_controller_params.linear.right_Kv,
    //                     vel_controller_params.linear.right_Ks,
    //                     true);
    // angular_kv_ks_tuner();
    // linear_ka_kp_ki_tuner(0.5_volt, 0.6, 2_sec);
    // angular_ka_kp_ki_tuner(0.5_volt, 0.6, 2_sec);

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
            pros::delay(30);
        }
    });

    // pros::delay(5000);
    // angular_kv_ks_tuner();
    path_follow_test();

    // TODO: add left/right control to vel controller directly
    // arc_pose_tracker.setPose({ -23.6_in, 0_in, 0_stDeg });

    // odom_offset_tuning();

    // while (true) {
    //     // Arcade control scheme
    //     // velocity_drivetrain.setBrakeMode(pros::MotorBrake::hold);
    //
    //     double dir = master.get_analog(ANALOG_LEFT_Y) / 127.0;
    //     double turn = -master.get_analog(ANALOG_RIGHT_X) / 127.0;
    //
    //     DifferentialSpeeds target { dir * max_velocity,
    //                                 turn * (max_velocity / 5.25_in) * rad };
    //     velocity_drivetrain.moveArcade(target.linear_velocity,
    //                                    target.angular_velocity);
    //
    //     if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    //         printDrivetrainData();
    //     }
    //
    //     pros::delay(20); // Run for 20 ms then update
    // }

    // using namespace lyfast::sysid;
    //
    // std::vector<DifferentialVoltageCommand> test_commands = {
    //     { 0.1_volt, 0.1_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.1_volt, -0.1_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 0.2_volt, 0.2_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.2_volt, -0.2_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 0.3_volt, 0.3_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.3_volt, -0.3_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 0.4_volt, 0.4_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.4_volt, -0.4_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 0.5_volt, 0.5_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.5_volt, -0.5_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 0.6_volt, 0.6_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.6_volt, -0.6_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 0.7_volt, 0.7_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.7_volt, -0.7_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 0.8_volt, 0.8_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.8_volt, -0.8_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 0.9_volt, 0.9_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -0.9_volt, -0.9_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { 1.0_volt, 1.0_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    //     { -1.0_volt, -1.0_volt, 1000_msec },
    //     { 0.0_volt, 0.0_volt, 2000_msec, false },
    // };
    // velocity_drivetrain.setBrakeMode(pros::MotorBrake::hold);
    //
    // auto data =
    //   lyfast::sysid::DifferentialUtils::generateData(test_commands,
    //                                                  velocity_drivetrain,
    //                                                  10_msec,
    //                                                  true);
    // velocity_drivetrain.moveTank(0_volt, 0_volt);
    // pros::delay(10000);
    // lyfast::sysid::DifferentialUtils::printData(data, 10_msec);

    // for (auto [left_voltage, right_voltage, duration, record] :
    // test_commands) {
    //     velocity_drivetrain.moveTank(left_voltage, right_voltage);
    //
    //     pros::delay(to_msec(duration));
    // }
    //
    // std::cout << "raw data: " << std::endl;
    // AngularMotorGroupUtils::printDataAsLatex(raw_data);
    //
    // std::cout << "filtered data: " << std::endl;
    // AngularMotorGroupUtils::printDataAsLatex(filtered_data);
    //
    // std::cout << "extra data (torque, current): " << std::endl;
    // printPairQuantitiesAsLatex(extra_data);
    //
    // std::cout << "power data: " << std::endl;
    // printQuantityVectorAsLatex(power_data);
    // std::cout << "tick based vel data: " << std::endl;
    // printQuantityVectorAsLatex(tick_based_vel_data);
}
