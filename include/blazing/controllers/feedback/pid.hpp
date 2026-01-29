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

    std::optional<Output> m_maxVoltage;

    std::optional<Input> previousError;
    Multiplied<Input, Time> integral = Multiplied<Input, Time>(0);

    std::optional<Time> previousTime = std::nullopt;

  public:
    PID(KP_t<Input, Output> kp,
        KI_t<Input, Output> ki,
        KD_t<Input, Output> kd,
        std::optional<Input> windupRange = std::nullopt,
        std::optional<Output> maxVoltage = std::nullopt)
        : m_kp(kp),
          m_ki(ki),
          m_kd(kd),
          m_windupRange(windupRange),
          m_maxVoltage(maxVoltage) {}

    PID(double kp,
        double ki,
        double kd,
        std::optional<double> windupRange = std::nullopt,
        std::optional<double> maxVoltage = std::nullopt,
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
          m_maxVoltage(
            maxVoltage.transform([outputUnits](double maxVoltage) -> Output {
                return maxVoltage * outputUnits;
            })) {}

    // motions don't call this since they always copy the object,
    // however any other usage does need to call it
    void reset() {
        integral = 0;
        previousTime = std::nullopt;
    }

    Output update(Input measurement, Input target, Time dt) {
        Input error = target - measurement;

        if (!previousError) previousError = error;

        const Divided<Input, Time> derivative =
          (dt != 0_sec) ? (error - *previousError) / dt :
                          Divided<Input, Time>(0);

        if (previousError)
            // use trapezoidal approximation if previous is available
            integral += (error + *previousError) * dt * 0.5;
        else
            // use Riemann sum approximation
            integral += error * dt;

        previousError = error;

        // sign flip reset. If the sign of error changes, set the integral to 0
        if (units::sgn(error) != units::sgn(*previousError))
            integral = Multiplied<Input, Time>(0);

        // anti windup range. Unless error is small enough, set the integral to
        // 0
        if (m_windupRange
              .transform([error](Input windupRange) {
                  return units::abs(error) > windupRange;
              })
              .value_or(false))
            integral = Multiplied<Input, Time>(0);

        Output result = error * m_kp + integral * m_ki + derivative * m_kd;

        if (m_maxVoltage) {
            result = units::clamp(result, -(*m_maxVoltage), *m_maxVoltage);
        }

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

    std::optional<Output> get_maxVoltage() {
        return m_maxVoltage;
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

    void set_positiveSlew(std::optional<Output> positiveSlew) {
        m_maxVoltage = positiveSlew;
    }

    // double versions
    void set_kp(double kp) {
        set_kp(kp * UKP);
    }

    void set_ki(double ki) {
        set_ki(ki * UKI);
    }

    void set_kd(double kd) {
        set_kd(kd * UKD);
    }

    void set_windupRange(std::optional<double> windupRange) {
        set_windupRange(windupRange.transform(
          [inputUnits = this->m_inputUnits](auto windupRange) -> Input {
              return windupRange * inputUnits;
          }));
    }

    void set_maxVoltage(std::optional<double> maxVoltage) {
        set_maxVoltage(maxVoltage.transform(
          [outputUnits = this->m_outputUnits](auto maxVoltage) -> Output {
              return maxVoltage * outputUnits;
          }));
    }
};
} // namespace blazing
