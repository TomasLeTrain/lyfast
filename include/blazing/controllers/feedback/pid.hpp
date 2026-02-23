#pragma once

#include "pros/rtos.hpp"
#include "units/units.hpp"
#include <optional>

namespace blazing {

template<typename Input, typename Output>
using KP_t = Divided<Output, Input>;
template<typename Input, typename Output>
using KI_t = Divided<Divided<Output, Time>, Input>;
template<typename Input, typename Output>
using KD_t = Divided<Multiplied<Output, Time>, Input>;

template<typename Input, typename Output>
class PID {
  public:
    // units used to convert doubles (since specyfing the units every time can
    // become annoying)
    Time m_timeUnits = 1_sec;
    Input m_inputUnits = Input(1);
    Output m_outputUnits = Output(1);

    KP_t<Input, Output> UKP { 1 };
    KI_t<Input, Output> UKI { 1 };
    KD_t<Input, Output> UKD { 1 };

  private:
    KP_t<Input, Output> m_kp;
    KI_t<Input, Output> m_ki;
    KD_t<Input, Output> m_kd;

    std::optional<Input> m_windupRange;
    std::optional<Output> m_maxOutput;
    std::optional<double> m_derivative_alpha;

    std::optional<Input> previousError;
    std::optional<Input> previousMeasurement;
    std::optional<Divided<Input, Time>> last_applied_derivative;
    Multiplied<Input, Time> integral = Multiplied<Input, Time>(0);

    std::optional<Time> previousTime = std::nullopt;

  public:
    PID(KP_t<Input, Output> kp,
        KI_t<Input, Output> ki,
        KD_t<Input, Output> kd,
        std::optional<Input> windupRange = std::nullopt,
        std::optional<Output> maxVoltage = std::nullopt,
        std::optional<double> derivative_alpha = std::nullopt)
        : m_kp(kp),
          m_ki(ki),
          m_kd(kd),
          m_windupRange(windupRange),
          m_maxOutput(maxVoltage),
          m_derivative_alpha(derivative_alpha) {}

    PID(double kp,
        double ki,
        double kd,
        std::optional<double> windupRange = std::nullopt,
        std::optional<double> maxVoltage = std::nullopt,
        std::optional<double> derivative_alpha = std::nullopt,
        Time timeUnits = 1_sec,
        Input inputUnits = Input(1),
        Output outputUnits = Output(1))
        : m_timeUnits(timeUnits),
          m_inputUnits(inputUnits),
          m_outputUnits(outputUnits),
          UKP(outputUnits / inputUnits),
          UKI(UKP / timeUnits),
          UKD(UKP * timeUnits),
          m_kp(kp * UKP),
          m_ki(ki * UKI),
          m_kd(kd * UKD),
          m_windupRange(
            windupRange.transform([inputUnits](double windupRange) -> Input {
                return windupRange * inputUnits;
            })),
          m_maxOutput(
            maxVoltage.transform([outputUnits](double maxVoltage) -> Output {
                return maxVoltage * outputUnits;
            })),
          m_derivative_alpha(derivative_alpha) {}

    // motions don't call this since they always copy the object,
    // however any other usage does need to call it
    void reset() {
        integral = 0;
        previousTime = std::nullopt;
    }

    Output update(Input measurement, Input target, Time dt) {
        Input error = target - measurement;

        if (!previousError) previousError = error;
        if (!previousMeasurement) previousMeasurement = measurement;

        // const Divided<Input, Time> curr_derivative =
        //   (dt != 0_sec) ? (error - *previousError) / dt :
        //                   Divided<Input, Time>(0);

        // TODO: test that moveto's don't break because of this
        const Divided<Input, Time> curr_derivative =
          (dt != 0_sec) ? (*previousMeasurement - measurement) / dt :
                          Divided<Input, Time>(0);

        Divided<Input, Time> applied_derivative = curr_derivative;

        if (m_derivative_alpha && last_applied_derivative) {
            // use low pass filter if wanted
            applied_derivative =
              curr_derivative * m_derivative_alpha.value() +
              *last_applied_derivative * (1 - m_derivative_alpha.value());
        }
        last_applied_derivative = applied_derivative;

        auto current_integral = integral;

        if (previousError)
            // use trapezoidal approximation if previous is available
            current_integral += (error + *previousError) * dt * 0.5;
        else
            // use Riemann sum approximation
            current_integral += error * dt;

        // sign flip reset. If the sign of error changes, set the integral
        // to 0
        if (units::sgn(error) != units::sgn(*previousError)) {
            current_integral = Multiplied<Input, Time>(0);
            current_integral += error * dt;

            // update integral regardless of saturation
            integral = current_integral;
        }

        previousError = error;
        previousMeasurement = measurement;

        // anti windup range. Unless error is small enough, set the integral
        // to
        // 0
        bool within_antiwindup_range =
          m_windupRange
            .transform([error](Input windupRange) {
                return units::abs(error) <= windupRange;
            })
            .value_or(true);

        // outside of windup range should be zero
        if (!within_antiwindup_range) {
            current_integral = Multiplied<Input, Time>(0);
            // update integral regardless of saturation
            integral = current_integral;
        }

        Output result =
          error * m_kp + current_integral * m_ki + applied_derivative * m_kd;

        if (
          // only apply saturation control if within windup range
          within_antiwindup_range &&
          // and max output defined
          m_maxOutput &&
          // and is saturating
          units::abs(result) >= *m_maxOutput &&
          // and saturation would be adding windup
          units::sgn(error) == units::sgn(result)) {
            // all conditions for saturation were met, don't update integral
        } else {
            // not saturating, update integral
            integral = current_integral;
        }

        // clamp result
        if (m_maxOutput)
            result = units::clamp(result, -(*m_maxOutput), *m_maxOutput);

        return result;
    }

    KP_t<Input, Output> get_kp() {
        return m_kp;
    }

    KI_t<Input, Output> get_ki() {
        return m_ki;
    }

    KD_t<Input, Output> get_kd() {
        return m_kd;
    }

    std::optional<Input> get_windupRange() {
        return m_windupRange;
    }

    std::optional<Output> get_maxOutput() {
        return m_maxOutput;
    }

    void set_kp(KP_t<Input, Output> kp) {
        m_kp = kp;
    }

    void set_ki(KI_t<Input, Output> ki) {
        m_ki = ki;
    }

    void set_kd(KD_t<Input, Output> kd) {
        m_kd = kd;
    }

    void set_windupRange(std::optional<Input> windupRange) {
        m_windupRange = windupRange;
    }

    void set_maxOutput(std::optional<Output> maxOutput) {
        m_maxOutput = maxOutput;
    }

    // double versions
    void set_kp(double kp) {
        m_kp = kp * UKP;
    }

    void set_ki(double ki) {
        m_ki = ki * UKI;
    }

    void set_kd(double kd) {
        m_kd = kd * UKD;
    }

    void set_windupRange(std::optional<double> windupRange) {
        m_windupRange = windupRange.transform(
          [inputUnits = this->m_inputUnits](auto windupRange) -> Input {
              return windupRange * inputUnits;
          });
    }

    void set_maxOutput(std::optional<double> maxOutput) {
        m_maxOutput = maxOutput.transform(
          [outputUnits = this->m_outputUnits](auto maxOutput) -> Output {
              return maxOutput * outputUnits;
          });
    }

    void set_derivativeAlpha(std::optional<double> derivative_alpha) {
        m_derivative_alpha = derivative_alpha;
    }
};
} // namespace blazing
