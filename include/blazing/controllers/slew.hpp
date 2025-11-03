#pragma once

#include "blazing/controllers/controllers.hpp"
#include "units/units.hpp"
#include <optional>
#include <variant>

namespace blazing {

class SlewController {
  public:
    using slew_t = Divided<Voltage, Time>;
    using slew_variant_t = std::optional<std::variant<slew_t, Voltage>>;

  private:
    std::optional<Voltage> last_output = std::nullopt;

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
            if (std::holds_alternative<Voltage>(variant)) {
                return std::get<Voltage>(variant) / delta_time;
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
    Voltage apply(Voltage output, Time delta_time) {
        if (!last_output) {
            last_output = 0_volt;
        }

        auto output_vel = delta_time == 0_sec ?
                            0.0_volt / sec :
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

        Voltage adjusted_output = *last_output + output_vel * delta_time;

        last_output = adjusted_output;
        return adjusted_output;
    }
};

class LinearSlewController : virtual ControllerBase {
  public:
    SlewController linear_slew;

    LinearSlewController(SlewController linear_slew)
        : linear_slew(linear_slew) {}

    LinearSlewController(
      SlewController::slew_variant_t accel_slew = std::nullopt,
      SlewController::slew_variant_t backwards_accel_slew = std::nullopt,
      SlewController::slew_variant_t decel_slew = std::nullopt,
      SlewController::slew_variant_t backwards_decel_slew = std::nullopt,
      Time delta_time = 10_msec)
        : linear_slew(accel_slew,
                      backwards_accel_slew,
                      decel_slew,
                      backwards_decel_slew,
                      delta_time) {}
};

class AngularSlewController : virtual ControllerBase {
  public:
    SlewController angular_slew;

    AngularSlewController(SlewController angular_slew)
        : angular_slew(angular_slew) {}

    AngularSlewController(
      SlewController::slew_variant_t accel_slew = std::nullopt,
      SlewController::slew_variant_t backwards_accel_slew = std::nullopt,
      SlewController::slew_variant_t decel_slew = std::nullopt,
      SlewController::slew_variant_t backwards_decel_slew = std::nullopt,
      Time delta_time = 10_msec)
        : angular_slew(accel_slew,
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

} // namespace blazing
