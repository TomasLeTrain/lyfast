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
    void set_linear_feedback(Controller new_linear_feedback) {
        this->linear_feedback = new_linear_feedback;
    }
};

template<typename Controller>
    requires Feedback<Controller, Length, Voltage>
struct LateralFeedbackController : virtual ControllerBase {
  public:
    Controller lateral_feedback;

    LateralFeedbackController(Controller lateral_feedback_controller)
        : lateral_feedback(lateral_feedback_controller) {}

    // creates a copy of the controller with different lateral feedback
    // controller
    void set_lateral_feedback(Controller new_lateral_feedback) {
        this->lateral_feedback = new_lateral_feedback;
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
    void set_angular_feedback(Controller new_angular_feedback) {
        this->angular_feedback = new_angular_feedback;
    }
};

template<typename Controller>
    requires Feedback<Controller, Length, LinearVelocity>
struct LinearVelocityFeedbackController : virtual ControllerBase {
  public:
    Controller linear_velocity_feedback;

    LinearVelocityFeedbackController(
      Controller linear_velocity_feedback_controller)
        : linear_velocity_feedback(linear_velocity_feedback_controller) {}

    void set_linear_velocity_feedback(Controller new_linear_velocity_feedback) {
        this->linear_velocity_feedback = new_linear_velocity_feedback;
    }
};

template<typename Controller>
    requires Feedback<Controller, Length, AngularVelocity>
struct LateralVelocityFeedbackController : virtual ControllerBase {
  public:
    Controller lateral_velocity_feedback;

    LateralVelocityFeedbackController(
      Controller lateral_velocity_feedback_controller)
        : lateral_velocity_feedback(lateral_velocity_feedback_controller) {}

    void
    set_lateral_velocity_feedback(Controller new_lateral_velocity_feedback) {
        this->lateral_velocity_feedback = new_lateral_velocity_feedback;
    }
};

template<typename Controller>
    requires Feedback<Controller, Angle, AngularVelocity>
struct AngularVelocityFeedbackController : virtual ControllerBase {
  public:
    Controller angular_velocity_feedback;

    AngularVelocityFeedbackController(Controller angular_velocity_feedback)
        : angular_velocity_feedback(angular_velocity_feedback) {}

    // creates a copy of the controller with different angular feedback
    // controller
    void
    set_angular_velocity_feedback(Controller new_angular_velocity_feedback) {
        this->angular_velocity_feedback = new_angular_velocity_feedback;
    }
};

using PIDLinearController = LinearFeedbackController<PID<Length, Voltage>>;
using PIDAngularController = AngularFeedbackController<PID<Angle, Voltage>>;

using PIDLinearVelocityController =
  LinearVelocityFeedbackController<PID<Length, LinearVelocity>>;
using PIDAngularVelocityController =
  AngularVelocityFeedbackController<PID<Angle, AngularVelocity>>;

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
concept hasLateralFeedback =
  Feedback<decltype(Controller::lateral_feedback), Length, Voltage>;

template<typename Controller>
concept hasAngularFeedback =
  Feedback<decltype(Controller::angular_feedback), Angle, Voltage>;

// Linear/Angular Velocity Feedback Concepts
template<typename Controller>
concept hasLinearVelocityFeedback =
  Feedback<decltype(Controller::linear_velocity_feedback),
           Length,
           LinearVelocity>;

template<typename Controller>
concept hasLateralVelocityFeedback =
  Feedback<decltype(Controller::linear_velocity_feedback),
           Length,
           AngularVelocity>;

template<typename Controller>
concept hasAngularVelocityFeedback =
  Feedback<decltype(Controller::angular_velocity_feedback),
           Angle,
           AngularVelocity>;

} // namespace blazing
