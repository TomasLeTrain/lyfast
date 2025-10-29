#pragma once

#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include <array>

namespace mp {
    class RobotConstraints {
        public:
        Length track_width;
        float coeff_friction;

        RobotConstraints(
            Length track_width,
            float coeff_friction
            ) :
            track_width(track_width),
            coeff_friction(coeff_friction)
        {}
    };

    class AngularConstraints {
        public:
        AngularVelocity max_angular_vel;
        AngularAcceleration max_angular_accel;
        AngularAcceleration max_angular_decel;

        AngularConstraints(
            AngularVelocity max_angular_vel,
            AngularAcceleration max_angular_accel,
            AngularAcceleration max_angular_decel
            ) :
            max_angular_vel(max_angular_vel),
            max_angular_accel(max_angular_accel),
            max_angular_decel(max_angular_decel)
        {}
    };

    class LinearConstraints {
        public:
        LinearVelocity max_vel;  
        LinearAcceleration max_accel;
        LinearAcceleration max_decel;

        LinearConstraints(
            LinearVelocity max_vel,  
            LinearAcceleration max_accel,
            LinearAcceleration max_decel
            ) :
            max_vel(max_vel),
            max_accel(max_accel),
            max_decel(max_decel)
        {}
    };


    
    // contains all the constraints required by the profile generator
    // since some constraints might not be changed for each trajectory,
    // we should have multiple ways to make the constraints 
    class Constraints {
        public:
            Length track_width;
            float coeff_friction;

            LinearVelocity max_vel;  
            LinearAcceleration max_accel;
            LinearAcceleration max_decel;

            AngularVelocity max_angular_vel;
            AngularAcceleration max_angular_accel;
            AngularAcceleration max_angular_decel;

            Constraints(
                Length track_width,
                float coeff_friction,

                LinearVelocity max_vel,  
                LinearAcceleration max_accel,
                LinearAcceleration max_decel,

                AngularVelocity max_angular_vel,
                AngularAcceleration max_angular_accel,
                AngularAcceleration max_angular_decel
            ) :
                track_width(track_width),
                coeff_friction(coeff_friction),

                max_vel(max_vel),
                max_accel(max_accel),
                max_decel(max_decel),

                max_angular_vel(max_angular_vel),
                max_angular_accel(max_angular_accel),
                max_angular_decel(max_angular_decel)
        {}

            Constraints(
                    RobotConstraints* robot_constraints,
                    LinearConstraints* linear_constraints,
                    AngularConstraints* angular_constraints
                    ) :
                    track_width(robot_constraints->track_width),
                    coeff_friction(robot_constraints->coeff_friction),

                    max_vel(linear_constraints->max_vel),
                    max_accel(linear_constraints->max_accel),
                    max_decel(linear_constraints->max_decel),

                    max_angular_vel(angular_constraints->max_angular_vel),
                    max_angular_accel(angular_constraints->max_angular_accel),
                    max_angular_decel(angular_constraints->max_angular_decel)
        {}
    };
}
