#pragma once

#include <concepts>
#include "units/units.hpp"

namespace blazing {

// Feedback Concept
template<typename Controller, typename Input, typename Output>
concept Feedback = requires(Controller controller,
                            Input measurement,
                            Input target,
                            Time duration) {
    {
        controller.update(measurement, target, duration)
    } -> std::same_as<Output>;
};
}; // namespace blazing
