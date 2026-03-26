#pragma once

#include "blazing/drivetrains/differential.hpp"
#include "blazing/utils.hpp"
#include "lyfast/plants/velocity_plants.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

// effectively acts as wrapper for the plant, only setting the target and
// forwarding plant data. Plant should be getting updated constantly elsewhere!
class VelocityDifferentialDrivetrain : public ChainableDrivetrain {
  protected:
    pros::MotorGroup* m_left_motors;
    pros::MotorGroup* m_right_motors;
    lyfast::DrivetrainVelocityPlant* m_plant;
    Length m_track_width;

    std::array<Voltage, 2> m_commanded_voltages { 0_volt, 0_volt };

  public:
    VelocityDifferentialDrivetrain(pros::MotorGroup* left_motors,
                                   pros::MotorGroup* right_motors,
                                   lyfast::DrivetrainVelocityPlant* plant,
                                   Length track_width)
        : m_left_motors(left_motors),
          m_right_motors(right_motors),
          m_plant(plant),
          m_track_width(track_width) {}

    // move robot based on left and right velocities
    void moveTank(Voltage left_voltage, Voltage right_voltage) {
        // TODO: saturate here or offload to controller?
        std::array<Voltage, 2> saturated_voltages { left_voltage,
                                                    right_voltage };

        // normalizes voltages to [-1, 1]
        auto [new_left_voltage, new_right_voltage] =
          desaturate(saturated_voltages, 1_volt);

        m_plant->setTarget(
          LeftRightVoltages { new_left_voltage, new_right_voltage });
    }

    void moveVoltages(std::vector<Voltage> voltages) override {
        // if voltages are invalid then the .at should throw an error
        moveTank(voltages.at(0), voltages.at(1));
    }

    // move robot based on left and right velocities
    // positive angular -> turns left
    void moveArcade(Voltage linear_output, Voltage angular_output) {
        moveTank(linear_output - angular_output,
                 linear_output + angular_output);
    }

    // move robot based on left and right velocities
    // positive angular -> turns left
    void moveArcade(LinearVelocity linear_velocity,
                    AngularVelocity angular_velocity) {
        // TODO: saturate here or offload to controller?
        m_plant->setTarget(
          DifferentialSpeeds { linear_velocity, angular_velocity });
    }

    // move robot based on left and right velocities
    void moveTank(LinearVelocity left_velocity, LinearVelocity right_velocity) {
        // convert left and right velocities into linear and angular
        // v = (v_l + v_r) / 2
        // w = (v_r - v_l) / (track_width)
        LinearVelocity v = (left_velocity + right_velocity) / 2;
        AngularVelocity w =
          rad * (right_velocity - left_velocity) / (m_track_width);
        moveArcade(v, w);
    }

    void setBrakeMode(pros::MotorBrake brake_mode) {
        m_left_motors->set_brake_mode_all(brake_mode);
        m_right_motors->set_brake_mode_all(brake_mode);
    }

    std::vector<Voltage> getVoltages() override {
        return { m_plant->getCommandedVoltages().left_voltage,
                 m_plant->getCommandedVoltages().right_voltage };
    }

    LeftRightSpeeds getDrivetrainVelocities() {
        return m_plant->getEstimatedSpeeds();
    }

    LeftRightVoltages getDrivetrainVoltages() {
        return LeftRightVoltages { get_group_voltage(m_left_motors),
                                   get_group_voltage(m_right_motors) };
    }
};
} // namespace lyfast
} // namespace blazing
