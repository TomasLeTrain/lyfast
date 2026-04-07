#pragma once

#include "blazing/utils.hpp"
#include "pros/device.hpp"
#include "pros/error.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <cmath>
#include <initializer_list>
#include <optional>
#include <variant>

namespace blazing {
enum TrackerOrientation {
    Sideways,
    Forwards
};

template<TrackerOrientation orientation>
class TrackingWheel {
  private:
    std::variant<pros::Rotation*, pros::MotorGroup*> sensor;
    Length offset;
    Length wheel_diameter;
    std::optional<AngularVelocity> final_rpm = std::nullopt;
    bool enabled = true;

    Length last_distance = INFINITY * m;
    Length m_delta = INFINITY * m;

  public:
    TrackingWheel(std::variant<pros::Rotation*, pros::MotorGroup*> sensor,
                  Length offset,
                  Length wheel_diameter,
                  std::optional<AngularVelocity> final_rpm = std::nullopt)
        : sensor(sensor),
          offset(offset),
          wheel_diameter(wheel_diameter),
          final_rpm(final_rpm) {}

    void setEnabled(bool enabled) {
        this->enabled = enabled;
    }

    void disable() {
        setEnabled(false);
    }

    void enable() {
        setEnabled(true);
    }

    Length getOffset() {
        return offset;
    }

    Length getDelta() {
        if (enabled)
            return m_delta;
        else
            return INFINITY * m;
    }

    void update() {
        auto motor_get_dist = [this](pros::MotorGroup* motors) -> Length {
            Length res = 0_in;
            double count = 0;
            for (auto position : motors->get_raw_position_all(NULL)) {
                if (position == PROS_ERR) continue;

                Number rotations =
                  (*final_rpm * static_cast<double>(position)) /
                  (3600_rpm * 50.0);

                res += rotations * (wheel_diameter * M_PI);
                count += 1.0;
            }

            Length current = INFINITY * m, delta = INFINITY * m;

            if (count != 0) {
                current = res / count;
            }

            if (std::isfinite(current.internal()) &&
                std::isfinite(last_distance.internal())) {
                delta = current - last_distance;
            }

            last_distance = current;
            return delta;
        };

        auto rotation_get_dist = [this](pros::Rotation* sensor) -> Length {
            if (sensor == nullptr || !sensor->is_installed()) {
                last_distance = INFINITY * m;
                return INFINITY * m;
            }

            Length current = INFINITY * m, delta = INFINITY * m;

            int32_t current_deg = sensor->get_position();

            if (current_deg != PROS_ERR) {
                const Length circumference = wheel_diameter * M_PI;
                current =
                  circumference * static_cast<double>(current_deg) / 36000.0;
            }

            if (current_deg != PROS_ERR &&
                std::isfinite(last_distance.internal())) {
                delta = current - last_distance;
            }

            last_distance = current;

            return delta;
        };

        if (std::holds_alternative<pros::MotorGroup*>(sensor)) {
            m_delta = motor_get_dist(std::get<pros::MotorGroup*>(sensor));
        } else {
            m_delta = rotation_get_dist(std::get<pros::Rotation*>(sensor));
        }
    }
};

using SidewaysTracker = TrackingWheel<TrackerOrientation::Sideways>;
using ForwardsTracker = TrackingWheel<TrackerOrientation::Forwards>;

class TrackingImu {
  private:
    double last_heading;
    Angle m_delta = INFINITY * rad;
    AngularVelocity m_angular_velocity = INFINITY * radps;
    pros::Imu* sensor;
    bool disabled = false;

  public:
    Angle getDelta() {
        return m_delta;
    }

    AngularVelocity getAngularVelocity() {
        return m_angular_velocity;
    }

    void update() {
        if (sensor == nullptr || !sensor->is_installed() || disabled) {
            last_heading = INFINITY;
            m_delta = Angle(INFINITY);
            m_angular_velocity = AngularVelocity(INFINITY);

            // gets disabled permanently if disconnects, since measurements from
            // now on are effectively useless
            disabled = true;
            return;
        }

        double current = sensor->get_rotation();
        double result = INFINITY;

        // disable if we get infinity
        if (!std::isfinite(current)) disabled = true;

        if (std::isfinite(current) && std::isfinite(last_heading)) {
            result = -1.0 * (current - last_heading);
        }

        last_heading = current;

        m_delta = from_stDeg(result);
        // specific to z down orientation
        m_angular_velocity = -from_degps(sensor->get_gyro_rate().z);
    }

    TrackingImu(pros::Imu* sensor)
        : sensor(sensor) {}
};

class ArcOdomTracker {
  private:
    std::vector<ForwardsTracker*> forwards_trackers;
    std::vector<SidewaysTracker*> sideways_trackers;

    std::vector<TrackingImu*> imus;
    units::V2Position m_tracking_center_offsets;
    bool m_logging;

    units::Pose pose {};
    Length forward_travel = 0_in;
    Length distance_traveled = 0_in;

    units::V2Position last_local_position_delta = { 0_in, 0_in };
    Angle last_heading_delta = 0_stDeg;

    units::V2Velocity velocity_vector;

    AngularVelocity angular_velocity;

    std::optional<Length> last_left_dist = std::nullopt;
    std::optional<Length> last_right_dist = std::nullopt;
    std::optional<Angle> last_heading = std::nullopt;
    std::optional<Time> last_time = std::nullopt;

    template<TrackerOrientation orientation>
    static std::optional<Angle> calculateWheelHeading(
      std::vector<TrackingWheel<orientation>*>& trackingWheels) {
        // check that there are enough tracking wheels
        if (trackingWheels.size() < 2) return std::nullopt;
        // get data
        for (size_t i = 0; i < trackingWheels.size(); i++) {
            const Length distance1 = trackingWheels.at(i)->getDelta();
            const Length offset1 = trackingWheels.at(i)->getOffset();

            if (!std::isfinite(distance1.internal())) continue;

            for (size_t j = i + 1; j < trackingWheels.size(); j++) {
                const Length distance2 = trackingWheels.at(j)->getDelta();
                const Length offset2 = trackingWheels.at(j)->getOffset();

                if (!std::isfinite(distance2.internal()) || offset1 == offset2)
                    continue;

                // return the calculated heading
                return from_stRad((distance1 - distance2) /
                                  (offset1 - offset2));
            }
        }
        return std::nullopt;
    }

    std::optional<Angle> getImuDeltaAngle() {
        Angle heading_delta = 0_stDeg;
        int imu_count = 0;

        for (auto& imu : imus) {
            Angle current = imu->getDelta();
            if (std::isfinite(current.internal())) {
                heading_delta += current;
                imu_count++;
            } else {
                if (m_logging) {
                    printf("imu returned infinity!\n");
                }
            }
        }

        if (imu_count == 0) return std::nullopt;

        return heading_delta / imu_count;
    }

    Time getTargetDeltaTime() {
        return 10_msec;
    }

  public:
    ArcOdomTracker(std::initializer_list<ForwardsTracker*> forwards_trackers,
                   std::initializer_list<SidewaysTracker*> sideways_trackers,
                   std::initializer_list<TrackingImu*> imus,
                   units::V2Position tracking_center_offsets = { 0_in, 0_in },
                   bool logging = false)
        : forwards_trackers(forwards_trackers),
          sideways_trackers(sideways_trackers),
          imus(imus),
          m_tracking_center_offsets(tracking_center_offsets),
          m_logging(logging) {}

    Angle getAngle() {
        return pose.orientation;
    }

    units::V2Position getPosition() {
        // add offsets to go from center of rotation to tracking center
        return pose + m_tracking_center_offsets.rotatedBy(getAngle());
    }

    LinearVelocity getLinearVelocity() {
        return velocity_vector.x;
    }

    LinearVelocity getTangentLinearVelocity() {
        return velocity_vector.y;
    }

    units::V2Velocity getLocalVelocityVector() {
        return velocity_vector;
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
        // subtract rotated offsets to get pose of center of rotation
        new_pose -= m_tracking_center_offsets.rotatedBy(getAngle());
        pose = new_pose;
    }

    void update() {
        bool first_update = !last_time;

        // NOTE: DON'T REMOVE!!! this updates last time to not be std::nullopt
        deltaTime(last_time);

        for (auto& tracker : imus) {
            tracker->update();
        }
        for (auto& tracker : sideways_trackers) {
            tracker->update();
        }
        for (auto& tracker : forwards_trackers) {
            tracker->update();
        }

        // almost guaranteed that all trackers are undefined,
        // just skip this update
        if (first_update) return;

        Angle heading_delta =
          getImuDeltaAngle()
            .or_else(
              std::bind(&calculateWheelHeading<Sideways>, sideways_trackers))
            .or_else(
              std::bind(&calculateWheelHeading<Forwards>, forwards_trackers))
            .value_or(0_stDeg);

        // default to zero
        units::V2Position deltas = units::origin<Length>,
                          offsets = units::origin<Length>;

        // TODO: when falling back to drivetrain make sure it uses both sides of
        // the drive instead of just one

        // use first tracker that gives good delta

        bool first_failed = false;
        double forwards_count = 0.0;

        for (auto& tracker : forwards_trackers) {
            Length current_delta = tracker->getDelta();
            if (!std::isfinite(current_delta.internal())) {
                if (m_logging) {
                    printf("forward tracker returned infinity!\n");
                }

                first_failed = true;
                continue;
            }
            deltas.x += current_delta;
            offsets.x += tracker->getOffset();
            forwards_count += 1.0;
            if (!first_failed) break;
            // break;
        }
        // if its 0.0 then deltas and such would be zero anyway
        if (first_failed && forwards_count != 0.0)
            deltas.x /= forwards_count, offsets.x /= forwards_count;

        for (auto& tracker : sideways_trackers) {
            Length current_delta = tracker->getDelta();
            if (!std::isfinite(current_delta.internal())) {
                if (m_logging) {
                    printf("sideways tracker returned infinity!\n");
                }
                continue;
            }
            deltas.y = current_delta;
            offsets.y = tracker->getOffset();
            break;
        }

        units::V2Position local_position_delta = [&] {
            if (heading_delta == 0_stDeg ||
                !std::isfinite(heading_delta.internal())) {
                return deltas;
            } else {
                // subtract to keep sign of offsets consistent with most libs
                return 2 * units::sin(heading_delta / 2) *
                       (deltas / to_stRad(heading_delta) - offsets);
            }
        }();

        // average of last two deltas to prevent noise due to polling rate
        velocity_vector = (last_local_position_delta + local_position_delta) /
                          (getTargetDeltaTime() * 2);

        angular_velocity =
          (last_heading_delta + heading_delta) / (getTargetDeltaTime() * 2);

        // uses imu measurement directly (if available)
        for (TrackingImu* imu : imus) {
            if (imu && std::isfinite(imu->getAngularVelocity().internal())) {
                angular_velocity = imu->getAngularVelocity();
                break;
            }
        }

        forward_travel += local_position_delta.x;
        // should be magnitude instead?
        distance_traveled += units::abs(local_position_delta.x);

        // update pose
        pose +=
          local_position_delta.rotatedBy(pose.orientation + heading_delta / 2);
        pose.orientation += heading_delta;

        last_local_position_delta = local_position_delta;
        last_heading_delta = heading_delta;
    }
};
} // namespace blazing
