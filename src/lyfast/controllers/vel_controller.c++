#include "lyfast/controllers/vel_controller.hpp"

namespace blazing {
namespace lyfast {

DifferentialSpeeds
desaturatePrioritizeAngularDiffSpeeds(DifferentialSpeeds target,
                                      Length track_width,
                                      LinearVelocity max_velocity) {
    Length track_radius = track_width / 2.0;

    // first determine how fast we want to go angular wise
    LinearVelocity lin_alg_target =
      (target.angular_velocity / rad) * track_radius;

    // clamp linear speed based on angular speed
    LinearVelocity max_lin_speed =
      units::abs(max_velocity) - units::abs(lin_alg_target);

    LinearVelocity new_lin_speed =
      units::clamp(target.linear_velocity, -max_lin_speed, max_lin_speed);

    return { new_lin_speed, target.angular_velocity };
}

DifferentialSpeeds desaturateDifferentialSpeeds(DifferentialSpeeds target,
                                                Length track_width,
                                                LinearVelocity max_velocity) {
    Length track_radius = track_width / 2.0;

    LinearVelocity target_left_vel =
      target.linear_velocity - (target.angular_velocity / rad) * track_radius;
    LinearVelocity target_right_vel =
      target.linear_velocity + (target.angular_velocity / rad) * track_radius;

    std::array<LinearVelocity, 2> saturated = { target_left_vel,
                                                target_right_vel };

    auto [new_left_vel, new_right_vel] =
      blazing::desaturate(saturated, max_velocity);

    LinearVelocity new_lin_vel = (new_left_vel + new_right_vel) / 2.0;
    AngularVelocity new_ang_vel =
      rad * (new_right_vel - new_left_vel) / track_width;

    return { new_lin_vel, new_ang_vel };
}

// TODO: these instantiations become invalid if header changes, should only live
// on the final project then?

// feedforward
// template struct FeedforwardVelocityControllerParams<LinearVelocity>;
// template struct FeedforwardVelocityControllerParams<AngularVelocity>;
//
// template class FeedforwardVelocityController<LinearVelocity>;
// template class FeedforwardVelocityController<AngularVelocity>;
//
// // PID
// template struct PIDVelocityControllerParams<LinearVelocity>;
// template struct PIDVelocityControllerParams<AngularVelocity>;
//
// template class PIDVelocityController<LinearVelocity>;
// template class PIDVelocityController<AngularVelocity>;
//
// // simple vel controller
// template class SimpleVelocityController<LinearVelocity>;
// template class SimpleVelocityController<AngularVelocity>;

} // namespace lyfast

} // namespace blazing
