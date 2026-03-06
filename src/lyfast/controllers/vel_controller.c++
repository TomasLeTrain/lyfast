#include "lyfast/controllers/vel_controller.hpp"

namespace blazing {
namespace lyfast {

// feedforward
template struct FeedforwardVelocityControllerParams<LinearVelocity>;
template struct FeedforwardVelocityControllerParams<AngularVelocity>;

template class FeedforwardVelocityController<LinearVelocity>;
template class FeedforwardVelocityController<AngularVelocity>;

// PID
template struct PIDVelocityControllerParams<LinearVelocity>;
template struct PIDVelocityControllerParams<AngularVelocity>;

template class PIDVelocityController<LinearVelocity>;
template class PIDVelocityController<AngularVelocity>;

// simple vel controller
template class SimpleVelocityController<LinearVelocity>;
template class SimpleVelocityController<AngularVelocity>;

} // namespace lyfast

} // namespace blazing
