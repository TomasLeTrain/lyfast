// #pragma once
//
// #include "lyfast/motion_profiling/constraints.h"
//
// #include "pros/rtos.h"
// #include "units/Angle.hpp"
// #include "units/Vector2D.hpp"
// #include "units/Pose.hpp"
// #include "units/units.hpp"
// #include <array>
// #include <cmath>
// #include <memory>
//
// #include "lyfast/geometry/curve.h"
//
// namespace motions {
//     namespace simple_mp {
//         class SimpleMotionPoint {
//             public:
//                 Time timestamp;
//                 Length position;
//                 LinearVelocity velocity;
//                 LinearAcceleration acceleration;
//
//                 SimpleMotionPoint(
//                     Time timestamp,
//                     Length position,
//                     LinearVelocity velocity,
//                     LinearAcceleration acceleration
//                     ) :
//                     timestamp(timestamp),
//                     position(position),
//                     velocity(velocity),
//                     acceleration(acceleration)
//                     { }
//         };
//
//         class TrapezoidalProfileTrajectory {
//             public:
//             std::vector<SimpleMotionPoint> points = {{0_sec,0_m,0_mps,0_mps2}};
//             LinearVelocity max_vel;
//             LinearAcceleration max_accel;
//             Time dt;
//             Length target;
//
//             Time accel_time, decel_time, cruise_time, decel_start_time;
//             Length accel_distance, decel_distance, cruise_distance, decel_start_distance;
//
//             void compute(){
//                 // We first find the amount of time it takes to accelerate to max velocity
//                 accel_time = max_vel / max_accel;
//
//                 // Using the acceleration time, we can find the distance it would take to reach max velocity
//                 accel_distance = max_vel * accel_time;
//
//                 // no time to cruise at the max speed, so we need to cap the max speed
//                 if (accel_distance  >  target / 2.0) {
//                     // target/2 = 1/2 * accel * t^2 
//                     // target = accel * t^2 
//                     // target/accel = t^2
//                     // t = sqrt(target/accel)
//                     accel_time = units::sqrt(target / max_accel);
//                     accel_distance = target / 2.0;
//                     max_vel = max_accel * accel_time;
//                 }
//
//                 // Since acceleration time = deceleration time in a trapezoidal profile, we can set these equal
//                 decel_time = accel_time;
//                 decel_distance = accel_distance;
//
//                 // Then calculate the cruising distance based on the distance left
//                 cruise_distance = target - accel_distance - decel_distance;
//                 cruise_time = cruise_distance  /  max_vel; // Divide by the velocity to get time
//
//                 // time at which we start to decelerate
//                 decel_start_time = accel_time + cruise_time; 
//                 decel_start_distance = accel_distance + cruise_distance; 
//             }
//             LinearVelocity getVelocity(Time t){
//                 // we are accelerating
//                 if (t  <  accel_time) {
//                     return (
//                             (max_vel  /  accel_time) // accel
//                             * t);
//                 }
//                 // we are crusing
//                 else  if (t < decel_start_time) {
//                     return  max_vel;
//                 }
//                 // if none of the above, we must be in deceleration
//                 else {
//                     return (
//                             max_vel - (
//                                 (max_vel / accel_time) // decel
//                                 * (t - decel_start_time)) );
//                 }
//             }
//             LinearVelocity getVelocity(Length distance){
//                 if (distance < accel_distance) {
//                     return 
//                         //((max_vel / accel_time) * t);
//                         units::sqrt(2 * distance *
//                             (max_vel / accel_time) // accel
//                             );
//                 }
//                 // we are crusing
//                 else  if (distance < decel_start_distance) {
//                     return  max_vel;
//                 }
//                 // if none of the above, we must be in deceleration
//                 else {
//                     return (
//                         max_vel - units::sqrt(2 *
//                                 (distance - decel_start_distance) *
//                                 (max_vel / accel_time) // decel
//                                 )
//                         );
//                 }
//             }
//
//             TrapezoidalProfileTrajectory(LinearVelocity max_vel,LinearAcceleration max_accel,Time dt, Length target)
//                 : max_vel(max_vel), max_accel(max_accel), dt(dt), target(target) {
//                     compute();
//                 }
//         };
//     }
// }
