#pragma once

#include "lyfast/controllers/vel_controller.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

template<typename Input>
class mpFeedback {
  public:
    using VelT = Divided<Input, Time>;
    using AccelT = Divided<VelT, Time>;

  private:
    VelT m_max_vel;
    AccelT m_max_accel;

    Input m_low_threshold_error;
    Divided<VelT, Input> m_low_threshold_kp;

  public:
    mpFeedback(VelT max_vel,
               AccelT max_accel,
               Input low_threshold_error,
               Divided<VelT, Input> low_threshold_kp)
        : m_max_vel(max_vel),
          m_max_accel(max_accel),
          m_low_threshold_error(low_threshold_error),
          m_low_threshold_kp(low_threshold_kp) {}

    VelT update(Input measurement, Input target, Time dt) {
        // signed error
        const Input error = target - measurement;
        const Number error_sgn = units::sgn(error);
        const Input abs_error = units::abs(error);

        const Length decel_dist = units::square(m_max_vel) / (2 * m_max_accel);

        if (abs_error < m_low_threshold_error) {
            // error is already signed
            return m_low_threshold_kp * error;
        } else if (abs_error > decel_dist) {
            return m_max_vel * error_sgn;
        } else {
            const Length length_left = decel_dist - abs_error;

            Exponentiated<VelT, std::ratio<2>> accel_term =
              (2 * m_max_accel * length_left);

            Exponentiated<VelT, std::ratio<2>> diff =
              units::square(m_max_vel) - accel_term;

            return units::sqrt(units::abs(diff)) * error_sgn;
        }
    }

    void setMaxVel(VelT max_vel) {
        m_max_vel = max_vel;
    }

    void setMaxAccel(AccelT max_accel) {
        m_max_accel = max_accel;
    }

    void setLowThresholdError(Input threshold) {
        m_low_threshold_error = threshold;
    }

    void setLowThresholdKp(Divided<VelT, Input> kp) {
        m_low_threshold_kp = kp;
    }

    VelT getMaxVel() {
        return m_max_vel;
    }

    AccelT getMaxAccel() {
        return m_max_accel;
    }

    Input getLowThresholdError() {
        return m_low_threshold_error;
    }

    Divided<VelT, Input> getLowThresholdKp() {
        return m_low_threshold_kp;
    }
};
} // namespace lyfast
} // namespace blazing
