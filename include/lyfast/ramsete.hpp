#pragma once

#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

namespace lyfast {

class Ramsete {
    void update() {
        units::V2Position curr_position;
        Angle curr_angle;
        units::Pose target;

        units::V2Position error =
          (target - curr_position).rotatedBy(curr_angle);

        AngularVelocity desiredAngularVelocity;
        LinearVelocity desiredVelocity;

        Angle errorAngle = angleDifference(target.orientation, curr_angle);

        const Divided<Number, Angle> zeta = 1 / rad;
        const Exponentiated<Divided<Angle, Length>, std::ratio<2>> beta =
          0.5 * units::pow<2>(rad / m);

        const Frequency k = 2.0f * zeta *
                            units::sqrt(units::square(desiredAngularVelocity) +
                                        beta * units::square(desiredVelocity));

        auto sinc = [](Angle angle) -> Divided<Number, Angle> {
            return units::sin(angle) / angle;
        };

        const LinearVelocity target_linearVelocity =
          units::cos(errorAngle) * desiredVelocity + k * error.x;
        const AngularVelocity target_angularVelocity =
          (desiredAngularVelocity + k * errorAngle +
           beta * desiredVelocity * sinc(errorAngle) * error.y);

    }
};

} // namespace lyfast
