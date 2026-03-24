#pragma once

#include "blazing/utils.hpp"
#include "lyfast/controllers/vel_controller.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <chrono>
#include <map>
#include <mutex>
#include <queue>

namespace blazing {
namespace lyfast {

template<typename State>
class VelocityEstimator {
  public:
    virtual State getPredictedState() = 0;

    virtual ~VelocityEstimator() = default;
};

} // namespace lyfast
} // namespace blazing
