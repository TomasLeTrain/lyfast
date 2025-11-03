#pragma once

#include "blazing/utils.hpp"
#include "pros/motor_group.hpp"
#include "units/units.hpp"
#include <array>
#include <concepts>

namespace blazing {

template<typename Q>
concept ArcadeDrivetrain =
  requires(Q q, Voltage linear_output, Voltage angular_output) {
      q.moveArcade(linear_output, angular_output);
  };
template<typename Q>
concept TankDrivetrain =
  requires(Q q, Voltage left_voltage, Voltage right_voltage) {
      q.moveTank(left_voltage, right_voltage);
  };

template<typename Q>
concept MotionChainableDrivetrain =
  requires(Q q, std::vector<Voltage> voltages, bool enabled) {
      { q.getEnabled() } -> std::same_as<bool>;
      { q.getVoltages() } -> std::same_as<std::vector<Voltage>>;
      q.moveVoltages(voltages);
      q.setEnabled(enabled);
  };

class ChainableDrivetrain {
  protected:
    bool enabled = true;

  public:
    void setEnabled(bool enabled) {
        this->enabled = enabled;
    }

    bool getEnabled() {
        return enabled;
    }

    virtual void moveVoltages(std::vector<Voltage> voltages) = 0;
    virtual std::vector<Voltage> getVoltages() = 0;
};

} // namespace blazing
