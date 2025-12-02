#pragma once

#include "blazing/utils.hpp"
#include "pros/device.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <iterator>
#include <optional>

namespace blazing {
class SimpleOdomTracker {
  private:
    pros::MotorGroup* left_motors;
    pros::MotorGroup* right_motors;
    pros::Imu* imu;

    Length track_width;
    Length wheel_diameter;
    AngularVelocity final_rpm;

    units::Pose pose {};
    Length forward_travel = 0_in;
    Length distance_traveled = 0_in;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;

    std::optional<Length> last_left_dist = std::nullopt;
    std::optional<Length> last_right_dist = std::nullopt;
    std::optional<Angle> last_heading = std::nullopt;
    std::optional<Time> last_time = std::nullopt;

  public:
    SimpleOdomTracker(pros::MotorGroup* left_motors,
                      pros::MotorGroup* right_motors,
                      pros::Imu* imu,
                      Length track_width,
                      Length wheel_diameter,
                      AngularVelocity final_rpm)
        : left_motors(left_motors),
          right_motors(right_motors),
          imu(imu),
          track_width(track_width),
          wheel_diameter(wheel_diameter),
          final_rpm(final_rpm) {}

    Angle getAngle() {
        return pose.orientation;
    }

    units::V2Position getPosition() {
        return pose;
    }

    LinearVelocity getLinearVelocity() {
        return linear_velocity;
    }

    AngularVelocity getAngularVelocity() {
        return angular_velocity;
    }

    Length getForwardTravel() {
        return forward_travel;
    }

    Length getDistanceTraveled() {
        return distance_traveled;
    }

    void setPose(units::Pose new_pose) {
        imu->set_rotation(to_cDeg(new_pose.orientation));
        pose = new_pose;
    }

    void update() {
        const Time delta_time = deltaTime(last_time);

        auto get_dist = [this](pros::MotorGroup* motors) -> Length {
            Length res = 0_in;
            double count = 0;
            for (auto position : motors->get_raw_position_all(NULL)) {
                // number of rotations
                Number rotations = (final_rpm * static_cast<double>(position)) /
                                   (3600_rpm * 50.0);
                res += rotations * (wheel_diameter * M_PI);
                count += 1.0;
            }
            return res / count;
        };

        const Length left_dist = get_dist(left_motors);
        const Length right_dist = get_dist(right_motors);

        if (!last_left_dist) last_left_dist = left_dist;
        if (!last_right_dist) last_right_dist = right_dist;

        const Length left_delta = left_dist - *last_left_dist;
        const Length right_delta = right_dist - *last_right_dist;

        last_left_dist = left_dist;
        last_right_dist = right_dist;

        const Length average_delta = (left_delta + right_delta) / 2;

        // NOTE: this is not super accurate, might return 0 due to the polling
        // rate
        linear_velocity = delta_time == 0_sec ? LinearVelocity(INFINITY) :
                                                average_delta / delta_time;

        forward_travel += average_delta;
        distance_traveled += units::abs(average_delta);

        const Angle heading = from_cDeg(imu->get_rotation());
        if (!last_heading) last_heading = heading;

        angular_velocity = imu->get_gyro_rate().z * degps;
        last_heading = heading;

        // update pose
        units::V2Position change_vector = {
            average_delta * units::cos(heading),
            average_delta * units::sin(heading)
        };

        pose = { pose + change_vector, heading };
    }
};
} // namespace blazing
