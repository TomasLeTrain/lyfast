#pragma once

#include "blazing/controllers/feedback/feedback.hpp"
#include "blazing/controllers/feedback/pid.hpp"
#include "blazing/controllers/feedforward/feedforward.hpp"
#include "blazing/trackers/tracker.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <concepts>

namespace blazing {

struct ControllerBase {};

// allows chaining controllers together
// really only works for feedback -> feedforward or feedforward -> feedforward
// since having feedback on the second controller would require having a
// measurement
template<typename Controller1,
         typename Controller2,
         typename Input,
         typename Intermediate,
         typename Output>
// first controller can be either feedback or feedforward
    requires(Feedforward<Controller1, Input, Intermediate> ||
             Feedback<Controller1, Input, Intermediate>) &&
            Feedforward<Controller2, Intermediate, Output>
struct CascadedControllers : virtual ControllerBase {
  public:
    Controller1 controller1;
    Controller2 controller2;

    CascadedControllers(Controller1 controller1, Controller2 controller2)
        : controller1(controller1),
          controller2(controller2) {}

    // feedback
    // only useful if first controller is feedback
    Output update(Input measurement, Input target, Time duration)
        requires Feedback<Controller1, Input, Intermediate>
    {
        Intermediate intermediate =
          controller1.update(measurement, target, duration);
        Output result2 = controller2.update(intermediate, duration);
        return result2;
    }

    // feedforward
    // only useful if first controller is feedforward
    Output update(Input target, Time duration)
        requires Feedforward<Controller1, Input, Intermediate>
    {
        Intermediate intermediate = controller1.update(target, duration);
        Output result = controller2.update(intermediate, duration);
        return result;
    }
};

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
  Feedback<decltype(Controller::linear_feedback), Length, Voltage>;

template<typename Controller>
concept hasAngularFeedback =
  Feedback<decltype(Controller::angular_feedback), Angle, Voltage>;

} // namespace blazing
