#pragma once

#include "blazing/controllers/controllers.hpp"

namespace blazing {

class VoltageClampController {
    std::optional<Voltage> max_voltage = std::nullopt;
    std::optional<Voltage> min_voltage = std::nullopt;

  public:
    VoltageClampController(std::optional<Voltage> max_voltage = std::nullopt,
                           std::optional<Voltage> min_voltage = std::nullopt)
        : max_voltage(max_voltage),
          min_voltage(min_voltage) {}

    void setMin(Voltage min) {
        this->min_voltage = min;
    }

    void setMax(Voltage max) {
        this->max_voltage = max;
    }

    Voltage applyMin(Voltage output) {
        auto clamp_func = units::max<Voltage, Voltage>;

        return min_voltage
          .transform([output, clamp_func](Voltage min_voltage) -> Voltage {
              return units::sgn(output) *
                     clamp_func(units::abs(output), min_voltage);
          })
          .value_or(output);
    }

    Voltage applyMax(Voltage output) {
        auto clamp_func = units::min<Voltage, Voltage>;

        return max_voltage
          .transform([output, clamp_func](Voltage max_voltage) -> Voltage {
              return units::sgn(output) *
                     clamp_func(units::abs(output), max_voltage);
          })
          .value_or(output);
    }

    // output should be signed, indicating its direction of travel
    Voltage apply(Voltage output) {
        output = applyMin(output);
        output = applyMax(output);

        return output;
    }
};

class LinearVoltageClampController : virtual ControllerBase {
  public:
    VoltageClampController linear_voltage_clamp;

    LinearVoltageClampController(VoltageClampController linear_voltage_clamp)
        : linear_voltage_clamp(linear_voltage_clamp) {}

    LinearVoltageClampController(
      std::optional<Voltage> max_voltage = std::nullopt,
      std::optional<Voltage> min_voltage = std::nullopt)
        : linear_voltage_clamp(max_voltage, min_voltage) {}
};

class AngularVoltageClampController : virtual ControllerBase {
  public:
    VoltageClampController angular_voltage_clamp;

    AngularVoltageClampController(VoltageClampController angular_voltage_clamp)
        : angular_voltage_clamp(angular_voltage_clamp) {}

    AngularVoltageClampController(
      std::optional<Voltage> max_voltage = std::nullopt,
      std::optional<Voltage> min_voltage = std::nullopt)
        : angular_voltage_clamp(max_voltage, min_voltage) {}
};

// voltage clamp constraints
template<typename Controller>
concept hasLinearVoltageClamp =
  requires(Controller controller) { controller.linear_voltage_clamp; };
template<typename Controller>
concept hasAngularVoltageClamp =
  requires(Controller controller) { controller.angular_voltage_clamp; };

} // namespace blazing
