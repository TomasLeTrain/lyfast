#pragma once

#include "blazing/controllers/feedback/feedback.hpp"
#include "blazing/controllers/feedback/pid.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <concepts>

namespace blazing {

struct ControllerBase {};

template<typename Controller>
    requires Feedback<Controller, Length, Voltage>
struct LinearFeedbackController : virtual ControllerBase {
  public:
    Controller linear_feedback;

    LinearFeedbackController(Controller linear_feedback_controller)
        : linear_feedback(linear_feedback_controller) {}

    // creates a copy of the controller with different linear feedback
    // controller
    template<typename Self>
    Self with_linear_feedback(this Self&& self,
                              Controller new_linear_feedback) {
        Self new_self = self;
        new_self.linear_feedback = new_linear_feedback;
        return new_self;
    }
};

template<typename Controller>
    requires Feedback<Controller, Angle, Voltage>
struct AngularFeedbackController : virtual ControllerBase {
  public:
    Controller angular_feedback;

    AngularFeedbackController(Controller angular_feedback)
        : angular_feedback(angular_feedback) {}

    // creates a copy of the controller with different angular feedback
    // controller
    template<typename Self>
    Self with_angular_feedback(this Self&& self,
                               Controller new_angular_feedback) {
        Self new_self = self;
        new_self.angular_feedback = new_angular_feedback;
        return new_self;
    }
};

using PIDLinearController = LinearFeedbackController<PID<Length, Voltage>>;
using PIDAngularController = AngularFeedbackController<PID<Angle, Voltage>>;

// inherits all the properties from the controllers being used
template<typename... ControllerTypes>
    requires(std::is_base_of_v<ControllerBase, ControllerTypes> && ...)
struct Controllers : virtual ControllerBase,
                     public ControllerTypes... {
  public:
    template<typename... U>
        requires(sizeof...(U) == sizeof...(ControllerTypes) &&
                 (std::is_constructible_v<ControllerTypes, U> && ...))
    Controllers(U&&... controllers)
        : ControllerTypes(std::forward<U>(controllers))... {}
};

// deduction guide allows specifying tolerance types from constructor
template<typename... ControllerTypes>
Controllers(ControllerTypes&&...)
  -> Controllers<std::remove_cvref_t<ControllerTypes>...>;

// Linear/Angular Feedback Concepts
template<typename Controller>
concept hasLinearFeedback =
  requires(Controller controller) { controller.linear_feedback; };

template<typename Controller>
concept hasAngularFeedback =
  requires(Controller controller) { controller.angular_feedback; };
} // namespace blazing
