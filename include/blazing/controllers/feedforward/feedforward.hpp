#pragma once

#include "units/units.hpp"
#include <concepts>

namespace blazing {

// Feedforward Concept
template<typename Controller, typename Input, typename Output>
concept Feedforward =
  requires(Controller controller, Input target, Time duration) {
      { controller.update(target, duration) } -> std::same_as<Output>;
  };

}; // namespace blazing
