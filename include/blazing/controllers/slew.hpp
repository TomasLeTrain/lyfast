#pragma once

#include "blazing/controllers/controllers.hpp"
#include "units/units.hpp"
#include <optional>
#include <variant>

namespace blazing {

template<typename T = Voltage>
class SlewController {
  public:
    using slew_t = Divided<T, Time>;
    using slew_variant_t = std::optional<std::variant<slew_t, T>>;

  private:
    std::optional<T> last_output = std::nullopt;

    Time targeted_delta_time = 10_msec;

    std::optional<slew_t> accel_slew;
    std::optional<slew_t> backwards_accel_slew;

    std::optional<slew_t> decel_slew;
    std::optional<slew_t> backwards_decel_slew;

    std::optional<slew_t> process_parameter(slew_variant_t optional_variant,
                                            Time delta_time) {
        if (!optional_variant)
            return std::nullopt;
        else {
            const auto& variant = optional_variant.value();
            if (std::holds_alternative<T>(variant)) {
                return std::get<T>(variant) / delta_time;
            } else {
                return std::get<slew_t>(variant);
            }
        }
    }

  public:
    SlewController(slew_variant_t accel_slew,
                   slew_variant_t backwards_accel_slew,
                   slew_variant_t decel_slew,
                   slew_variant_t backwards_decel_slew,
                   Time delta_time = 10_msec)
        : targeted_delta_time(delta_time),
          accel_slew(process_parameter(accel_slew, delta_time)),
          backwards_accel_slew(
            process_parameter(backwards_accel_slew, delta_time)),
          decel_slew(process_parameter(decel_slew, delta_time)),
          backwards_decel_slew(
            process_parameter(backwards_decel_slew, delta_time)) {}

    void set_accel(slew_variant_t accel_slew) {
        this->accel_slew = process_parameter(accel_slew, targeted_delta_time);
    }

    void set_backwards_accel(slew_variant_t backwards_accel_slew) {
        this->backwards_accel_slew =
          process_parameter(backwards_accel_slew, targeted_delta_time);
    }

    void set_decel(slew_variant_t decel_slew) {
        this->decel_slew = process_parameter(decel_slew, targeted_delta_time);
    }

    void set_backwards_decel(slew_variant_t backwards_decel_slew) {
        this->backwards_decel_slew =
          process_parameter(backwards_decel_slew, targeted_delta_time);
    }

    // output should be signed, indicating its direction of travel
    T apply(T output, Time delta_time) {
        if (!last_output) {
            last_output = T(0.0);
        }

        auto output_vel = delta_time == 0_sec ?
                            T(0.0) / sec :
                            (output - *last_output) / delta_time;

        bool accelerating = units::sgn(output) == units::sgn(output_vel);
        bool forwards = units::sgn(output) >= 0.0;

        auto applySlew = [&](std::optional<slew_t> slew) {
            output_vel = units::sgn(output_vel) *
                         units::min(units::abs(output_vel), *slew);
        };

        // decelerating
        if (!accelerating) {
            if (!forwards && backwards_decel_slew) {
                // backwards decel set, use
                applySlew(backwards_decel_slew);
            } else if (decel_slew) {
                // use decel regardless of direction
                applySlew(decel_slew);
            }
        } else if (accelerating) {
            if (!forwards && backwards_accel_slew) {
                // backwards decel set, use
                applySlew(backwards_accel_slew);
            } else if (accel_slew) {
                // use accel regardless of direction
                applySlew(accel_slew);
            }
        }

        T adjusted_output = *last_output + output_vel * delta_time;

        last_output = adjusted_output;
        return adjusted_output;
    }
};

class LinearSlewController : virtual ControllerBase {
  public:
    SlewController<Voltage> linear_slew;

    LinearSlewController(SlewController<Voltage> linear_slew)
        : linear_slew(linear_slew) {}

    LinearSlewController(
      SlewController<Voltage>::slew_variant_t accel_slew = std::nullopt,
      SlewController<Voltage>::slew_variant_t backwards_accel_slew =
        std::nullopt,
      SlewController<Voltage>::slew_variant_t decel_slew = std::nullopt,
      SlewController<Voltage>::slew_variant_t backwards_decel_slew =
        std::nullopt,
      Time delta_time = 10_msec)
        : linear_slew(accel_slew,
                      backwards_accel_slew,
                      decel_slew,
                      backwards_decel_slew,
                      delta_time) {}
};

class AngularSlewController : virtual ControllerBase {
  public:
    SlewController<Voltage> angular_slew;

    AngularSlewController(SlewController<Voltage> angular_slew)
        : angular_slew(angular_slew) {}

    AngularSlewController(
      SlewController<Voltage>::slew_variant_t accel_slew = std::nullopt,
      SlewController<Voltage>::slew_variant_t backwards_accel_slew =
        std::nullopt,
      SlewController<Voltage>::slew_variant_t decel_slew = std::nullopt,
      SlewController<Voltage>::slew_variant_t backwards_decel_slew =
        std::nullopt,
      Time delta_time = 10_msec)
        : angular_slew(accel_slew,
                       backwards_accel_slew,
                       decel_slew,
                       backwards_decel_slew,
                       delta_time) {}
};

class LinearVelocitySlewController : virtual ControllerBase {
  public:
    SlewController<LinearVelocity> linear_velocity_slew;

    LinearVelocitySlewController(
      SlewController<LinearVelocity> linear_velocity_slew)
        : linear_velocity_slew(linear_velocity_slew) {}

    LinearVelocitySlewController(
      SlewController<LinearVelocity>::slew_variant_t accel_slew = std::nullopt,
      SlewController<LinearVelocity>::slew_variant_t backwards_accel_slew =
        std::nullopt,
      SlewController<LinearVelocity>::slew_variant_t decel_slew = std::nullopt,
      SlewController<LinearVelocity>::slew_variant_t backwards_decel_slew =
        std::nullopt,
      Time delta_time = 10_msec)
        : linear_velocity_slew(accel_slew,
                               backwards_accel_slew,
                               decel_slew,
                               backwards_decel_slew,
                               delta_time) {}
};

class AngularVelocitySlewController : virtual ControllerBase {
  public:
    SlewController<AngularVelocity> angular_velocity_slew;

    AngularVelocitySlewController(
      SlewController<AngularVelocity> angular_velocity_slew)
        : angular_velocity_slew(angular_velocity_slew) {}

    AngularVelocitySlewController(
      SlewController<AngularVelocity>::slew_variant_t accel_slew = std::nullopt,
      SlewController<AngularVelocity>::slew_variant_t backwards_accel_slew =
        std::nullopt,
      SlewController<AngularVelocity>::slew_variant_t decel_slew = std::nullopt,
      SlewController<AngularVelocity>::slew_variant_t backwards_decel_slew =
        std::nullopt,
      Time delta_time = 10_msec)
        : angular_velocity_slew(accel_slew,
                                backwards_accel_slew,
                                decel_slew,
                                backwards_decel_slew,
                                delta_time) {}
};

template<typename Controller>
concept hasLinearSlew =
  requires(Controller controller) { controller.linear_slew; };
template<typename Controller>
concept hasAngularSlew =
  requires(Controller controller) { controller.angular_slew; };

template<typename Controller>
concept hasLinearVelocitySlew =
  requires(Controller controller) { controller.linear_velocity_slew; };
template<typename Controller>
concept hasAngularVelocitySlew =
  requires(Controller controller) { controller.angular_velocity_slew; };

} // namespace blazing
