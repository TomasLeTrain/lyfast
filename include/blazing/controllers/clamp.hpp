#pragma once

#include "blazing/controllers/controllers.hpp"
#include "units/units.hpp"

namespace blazing {

template<typename T>
class ClampController {
    std::optional<T> m_max = std::nullopt;
    std::optional<T> m_min = std::nullopt;

  public:
    ClampController(std::optional<T> max = std::nullopt,
                    std::optional<T> min = std::nullopt)
        : m_max(max),
          m_min(min) {}

    void setMin(T min) {
        this->m_min = min;
    }

    void setMax(T max) {
        this->m_max = max;
    }

    T applyMin(T output) {
        auto clamp_func = units::max<T, T>;

        return m_min
          .transform([output, clamp_func](T m_min) -> T {
              return units::sgn(output) * clamp_func(units::abs(output), m_min);
          })
          .value_or(output);
    }

    T applyMax(T output) {
        auto clamp_func = units::min<T, T>;

        return m_max
          .transform([output, clamp_func](T m_max) -> T {
              return units::sgn(output) * clamp_func(units::abs(output), m_max);
          })
          .value_or(output);
    }

    // output should be signed, indicating its direction of travel
    T apply(T output) {
        output = applyMin(output);
        output = applyMax(output);

        return output;
    }
};

class LinearVoltageClampController : virtual ControllerBase {
  public:
    ClampController<Voltage> linear_voltage_clamp;

    LinearVoltageClampController(ClampController<Voltage> linear_voltage_clamp)
        : linear_voltage_clamp(linear_voltage_clamp) {}

    LinearVoltageClampController(
      std::optional<Voltage> max_voltage = std::nullopt,
      std::optional<Voltage> min_voltage = std::nullopt)
        : linear_voltage_clamp(max_voltage, min_voltage) {}
};

class AngularVoltageClampController : virtual ControllerBase {
  public:
    ClampController<Voltage> angular_voltage_clamp;

    AngularVoltageClampController(
      ClampController<Voltage> angular_voltage_clamp)
        : angular_voltage_clamp(angular_voltage_clamp) {}

    AngularVoltageClampController(
      std::optional<Voltage> max_voltage = std::nullopt,
      std::optional<Voltage> min_voltage = std::nullopt)
        : angular_voltage_clamp(max_voltage, min_voltage) {}
};

class LinearVelocityClampController : virtual ControllerBase {
  public:
    ClampController<LinearVelocity> linear_velocity_clamp;

    LinearVelocityClampController(
      ClampController<LinearVelocity> linear_velocity_clamp)
        : linear_velocity_clamp(linear_velocity_clamp) {}

    LinearVelocityClampController(
      std::optional<LinearVelocity> max_vel = std::nullopt,
      std::optional<LinearVelocity> min_vel = std::nullopt)
        : linear_velocity_clamp(max_vel, min_vel) {}
};

class AngularVelocityClampController : virtual ControllerBase {
  public:
    ClampController<AngularVelocity> angular_velocity_clamp;

    AngularVelocityClampController(
      ClampController<AngularVelocity> angular_velocity_clamp)
        : angular_velocity_clamp(angular_velocity_clamp) {}

    AngularVelocityClampController(
      std::optional<AngularVelocity> max_vel = std::nullopt,
      std::optional<AngularVelocity> min_vel = std::nullopt)
        : angular_velocity_clamp(max_vel, min_vel) {}
};

// voltage clamp constraints
template<typename Controller>
concept hasLinearVoltageClamp =
  requires(Controller controller) { controller.linear_voltage_clamp; };
template<typename Controller>
concept hasAngularVoltageClamp =
  requires(Controller controller) { controller.angular_voltage_clamp; };

template<typename Controller>
concept hasLinearVelocityClamp =
  requires(Controller controller) { controller.linear_velocity_clamp; };
template<typename Controller>
concept hasAngularVelocityClamp =
  requires(Controller controller) { controller.angular_velocity_clamp; };

} // namespace blazing
