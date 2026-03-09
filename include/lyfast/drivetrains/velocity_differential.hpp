#pragma once

#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "lyfast/plants/velocity_plants.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

// effectively acts as wrapper for the plant, only setting the target and
// forwarding plant data. Plant should be getting updated constantly elsewhere!
class VelocityDifferentialDrivetrain : public ChainableDrivetrain {
  protected:
    pros::MotorGroup* left_motors;
    pros::MotorGroup* right_motors;

    lyfast::DrivetrainVelocityPlant* plant;

    std::array<Voltage, 2> voltages { 0_volt, 0_volt };

  public:
    VelocityDifferentialDrivetrain(Length track_width)
        : m_track_width(track_width) {}

    void moveVoltages(std::vector<Voltage> voltages) override {
        // if voltages are invalid then the .at should throw an error
        moveTank(voltages.at(0), voltages.at(1));
    }

    std::vector<Voltage> getVoltages() override {
        return { voltages.at(0), voltages.at(1) };
    }

    // move robot based on left and right velocities
    void moveTank(Voltage left_voltage, Voltage right_voltage) {
        // set voltages vector regardless of hardware action
        voltages = { left_voltage, right_voltage };

        // return if not doing hardware action
        if (!enabled) {
            return;
        }

        if (left_motors != nullptr && right_motors != nullptr) {
            left_motors->move_voltage(to_mvolt(12 * left_voltage));
            right_motors->move_voltage(to_mvolt(12 * right_voltage));
        }
    }

    // move robot based on left and right velocities
    // positive angular -> turns left
    void moveArcade(Voltage linear_output, Voltage angular_output) {
        std::array<Voltage, 2> saturated_voltages {
            linear_output - angular_output,
            linear_output + angular_output
        };

        // normalizes voltages to [-1, 1]
        auto [left_voltage, right_voltage] =
          desaturate(saturated_voltages, 1_volt);

        moveTank(left_voltage, right_voltage);
    }

    // move robot based on left and right velocities
    void moveTank(LinearVelocity left_voltage, LinearVelocity right_voltage) {
        plant.resetController();
        // TODO: have from last update time?
        // plant update should be getting called always, this should just set
        // the target
        plant.controllerUpdate(DifferentialSpeeds {}, 10_msec);
    }

    // move robot based on left and right velocities
    // positive angular -> turns left
    void moveArcade(Voltage linear_output, Voltage angular_output) {
        std::array<Voltage, 2> saturated_voltages {
            linear_output - angular_output,
            linear_output + angular_output
        };

        // normalizes voltages to [-1, 1]
        auto [left_voltage, right_voltage] =
          desaturate(saturated_voltages, 1_volt);

        moveTank(left_voltage, right_voltage);
    }

    void setBrakeMode(pros::MotorBrake brake_mode) {
        left_motors->set_brake_mode_all(brake_mode);
        right_motors->set_brake_mode_all(brake_mode);
    }

    LeftRightSpeeds getDrivetrainVelocities() {
        return plant.getEstimatedSpeeds();
    }

    LeftRightVoltages getDrivetrainVoltages() {
        return LeftRightVoltages { get_group_voltage(left_motors),
                                   get_group_voltage(right_motors) };
    }

    LinearVelocity getMaxVelocity() {
        // v = r * omega
        return (wheel_diameter / 2) * final_rpm / rad;
    }
};
} // namespace lyfast
} // namespace blazing
