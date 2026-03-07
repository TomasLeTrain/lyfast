#pragma once

#include "Eigen/Core"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <array>

namespace blazing {
namespace lyfast {

// returns -1 if gearing is invalid
// TODO: move to blazing utils?
inline AngularVelocity gearingToVelocity(pros::MotorGears gearing) {
    if (gearing == pros::MotorGears::rpm_600)
        return 600_rpm;
    else if (gearing == pros::MotorGears::rpm_200)
        return 200_rpm;
    else if (gearing == pros::MotorGears::rpm_100)
        return 100_rpm;
    // if encoder units are not set then it defaults to 200?
    return 200_rpm;
}

class MotorGroupKalmanFilter {
  public:
    using VelCovarianceUnit = Exponentiated<AngularVelocity, std::ratio<2>>;
    using AccelCovarianceUnit =
      Exponentiated<AngularAcceleration, std::ratio<2>>;

    struct Input {
        Voltage voltage;
    };

    struct State {
        AngularVelocity velocity;
        AngularAcceleration acceleration;
    };

    struct Constants {
        // the supposed final rpm after applying gearing. For example:
        // 600 rpm goes through 2:1 gear ratio, now becomes 1200 rpm (final
        // gearing rpm)
        AngularVelocity final_gearing_rpm;

        // kv constant of the motor
        Divided<Voltage, AngularVelocity> Kv;
        // ka constant of the motor
        Divided<Voltage, AngularAcceleration> Ka;
        // ks constant of the motor
        Voltage Ks;

        AccelCovarianceUnit accel_process_covariance = units::square(5_rpm2);

        // cov = factor * measurement^2
        // covariance increases on larger velocities
        float measurement_covariance_factor = units::square(0.4);
        // minimum covariance
        VelCovarianceUnit measurement_covariance_offset = units::square(5_rpm);
    };

  private:
    pros::MotorGroup* motor_group;

    Constants m_constants;
    State m_state_estimate;
    Input m_input;
    VelCovarianceUnit m_vel_covariance { 0 };
    AccelCovarianceUnit m_accel_covariance { 0 };

    void correctSingleMotor(std::uint8_t idx) {
        AngularVelocity motor_reported_velocity =
          motor_group->get_actual_velocity(idx) * rpm;
        pros::MotorGears gearing = motor_group->get_gearing(idx);

        AngularVelocity geared_motor_reported_velocity =
          (motor_reported_velocity / gearingToVelocity(gearing)) *
          m_constants.final_gearing_rpm;

        std::cout << "motor vel: "
                  << geared_motor_reported_velocity.convert(radps) << std::endl;

        // TODO: perform basic filtering on the reported velocity?

        AngularVelocity innovation =
          geared_motor_reported_velocity - m_state_estimate.velocity;
        std::cout << "innovation: " << innovation.internal() << std::endl;

        VelCovarianceUnit measurement_covariance =
          m_constants.measurement_covariance_factor *
            units::square(geared_motor_reported_velocity) +
          m_constants.measurement_covariance_offset;
        std::cout << "meas cov: " << measurement_covariance.internal()
                  << std::endl;

        VelCovarianceUnit innovation_covariance =
          m_vel_covariance + measurement_covariance;
        std::cout << "innovation cov: " << innovation_covariance.internal()
                  << std::endl;

        float gain = m_vel_covariance / innovation_covariance;
        std::cout << "gain: " << gain << std::endl;

        m_state_estimate.velocity =
          m_state_estimate.velocity + gain * innovation;
        std::cout << "new vel: " << m_state_estimate.velocity << std::endl;

        // update covariance from measurement
        m_vel_covariance = (1 - gain) * m_vel_covariance;
        std::cout << "new cov: " << m_vel_covariance << std::endl;
        std::cout << "ENDED" << std::endl;
    }

    // update input (voltage) based on motor data
    void updateInput() {
        m_input.voltage = 0_volt;
        int motor_count = 0;
        for (auto voltage : motor_group->get_voltage_all()) {
            m_input.voltage += from_mvolt(voltage) / 12.0;
            motor_count++;
        }
        m_input.voltage /= static_cast<float>(motor_count);
        std::cout << "input updated to " << m_input.voltage << std::endl;
    }

  public:
    // take measurements from the motor(s) and correct model based on them
    void correct() {
        // apply correction for each motor
        std::cout << "correcting" << std::endl;
        for (int i = 0; i < motor_group->size(); i++) {
            correctSingleMotor(i);
        }
    }

    // predict x_k+1 from x_k and u_k
    void predict(Time dt) {
        // update input after prediction
        updateInput();

        Voltage V_eff;
        if (units::abs(m_input.voltage) < m_constants.Ks) {
            // voltage low enough that it cannot overcome friction, resulting in
            // no movement
            V_eff = 0_volt;
        } else {
            V_eff =
              m_input.voltage - m_constants.Ks * units::sgn(m_input.voltage);
        }
        std::cout << "START" << std::endl;
        std::cout << "V_eff " << V_eff << std::endl;

        // V = kv * v + ka * a + ks * sgn(V)
        // [V - sgn(v) * ks] = kv * v + ka * a
        //
        // V_eff = [V - sgn(v) * ks]
        //
        // V_eff = kv * v + ka * a
        //
        // a = (V_eff - kv * v) / ka
        // a = V_eff * 1/ka - (kv/ka) * v
        // a = V_eff * (1/ka) + (-kv/ka) * v
        //
        // A = - kv / ka
        // B = 1/ka
        //
        // a = A * v + B * V_eff

        Frequency A = -(m_constants.Kv / m_constants.Ka);
        auto B = (1.0 / m_constants.Ka);

        std::cout << "A/B " << A << " " << B << std::endl;

        AngularAcceleration new_predicted_a =
          m_state_estimate.velocity * A + B * V_eff;

        AngularVelocity new_predicted_v =
          m_state_estimate.velocity + m_state_estimate.acceleration * dt;

        std::cout << "new predicted v " << new_predicted_v << std::endl;

        m_state_estimate = { .velocity = new_predicted_v,
                             .acceleration = new_predicted_a };

        m_vel_covariance = m_vel_covariance + dt * dt * m_accel_covariance;
        // NOTE: accel updated after vel so the update can be done in place
        m_accel_covariance =
          m_accel_covariance + m_constants.accel_process_covariance;

        Multiplied<Exponentiated<Frequency, std::ratio<2>>,
                   Exponentiated<AngularAcceleration, std::ratio<2>>>
          what = A * A * m_accel_covariance;

        std::cout << "new predict covairance " << m_vel_covariance << std::endl;
    }

    Input getInput() {
        return m_input;
    }

    State getPredictedState() {
        return m_state_estimate;
    }

    void setPredictedState(State new_state,
                           VelCovarianceUnit vel_covariance,
                           AccelCovarianceUnit accel_covariance) {
        m_state_estimate = new_state;
        m_vel_covariance = vel_covariance;
        m_accel_covariance = accel_covariance;
    }

    // resets the state and covariances to all zero
    void reset() {
        MotorGroupKalmanFilter::State state { .velocity = 0_radps,
                                              .acceleration = 0_radps2 };
        MotorGroupKalmanFilter::VelCovarianceUnit vel_covariance {
            units::square(0_rpm)
        };

        MotorGroupKalmanFilter::AccelCovarianceUnit accel_covariance {
            units::square(0_rpm2)
        };
        setPredictedState(state, vel_covariance, accel_covariance);
    }

    MotorGroupKalmanFilter(
      pros::MotorGroup* motors,
      Constants constants,
      State initial_state_estimate = State { .velocity = 0_radps,
                                             .acceleration = 0_radps2 },
      VelCovarianceUnit vel_covariance = VelCovarianceUnit { 0 },
      AccelCovarianceUnit accel_covariance = AccelCovarianceUnit { 0 })
        : motor_group(motors),
          m_constants(constants),
          m_state_estimate(initial_state_estimate),
          m_input({ .voltage = 0_volt }),
          m_vel_covariance(vel_covariance),
          m_accel_covariance(accel_covariance) {}
};

template<int States, int Inputs, int Outputs>
class KalmanFilter {
  public:
    using StateVector = std::array<float, States>;
    using InputVector = std::array<float, Inputs>;
    using OutputVector = std::array<float, Outputs>;

    using StateArray = std::array<float, States>;
    using OutputArray = std::array<float, Outputs>;

    using StateMatrix = Eigen::Matrix<float, States, States>;

    /**
     * Constructs a Kalman filter with the given plant.
     *
     * See
     * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#process-and-measurement-noise-covariance-matrices
     * for how to select the standard deviations.
     *
     * @param plant              The plant used for the prediction step.
     * @param stateStdDevs       Standard deviations of model states.
     * @param measurementStdDevs Standard deviations of measurements.
     * @param dt                 Nominal discretization timestep.
     * @throws std::invalid_argument If the system is undetectable.
     */
    KalmanFilter(const StateArray& stateStdDevs,
                 const OutputArray& measurementStdDevs,
                 Time dt) {
        m_contQ = MakeCovMatrix(stateStdDevs);
        m_contR = MakeCovMatrix(measurementStdDevs);
        m_dt = dt;

        Eigen::Matrix<float, States, States> A;
        Eigen::Matrix<float, States, States> plant_C;

        // Find discrete A and Q
        Eigen::Matrix<float, States, States> discA;
        Eigen::Matrix<float, States, States> discQ;
        DiscretizeAQ<States>(A, m_contQ, dt, &discA, &discQ);

        Eigen::Matrix<float, Outputs, Outputs> discR =
          DiscretizeR<Outputs>(m_contR, dt);

        const auto& C = plant_C;

        if (auto P = DARE<States, Outputs>(discA.transpose(),
                                           C.transpose(),
                                           discQ,
                                           discR)) {
            m_initP = P.value();
        } else {
            // TODO: log error
        }

        Reset();
    }

    /**
     * Returns the error covariance matrix P.
     */
    const StateMatrix& P() const {

        return m_P;
    }

    /**
     * Returns an element of the error covariance matrix P.
     *
     * @param i Row of P.
     * @param j Column of P.
     */
    double P(int i, int j) const {
        return m_P(i, j);
    }

    /**
     * Set the current error covariance matrix P.
     *
     * @param P The error covariance matrix P.
     */
    void SetP(const StateMatrix& P) {
        m_P = P;
    }

    /**
     * Returns the state estimate x-hat.
     */
    const StateVector& Xhat() const {
        return m_xHat;
    }

    /**
     * Returns an element of the state estimate x-hat.
     *
     * @param i Row of x-hat.
     */
    double Xhat(int i) const {
        return m_xHat(i);
    }

    /**
     * Set initial state estimate x-hat.
     *
     * @param xHat The state estimate x-hat.
     */
    void SetXhat(const StateVector& xHat) {
        m_xHat = xHat;
    }

    /**
     * Set an element of the initial state estimate x-hat.
     *
     * @param i     Row of x-hat.
     * @param value Value for element of x-hat.
     */
    void SetXhat(int i, double value) {
        m_xHat(i) = value;
    }

    /**
     * Resets the observer.
     */
    void Reset() {
        m_xHat.setZero();
        m_P = m_initP;
    }

    /**
     * Project the model into the future with a new control input u.
     *
     * @param u  New control input from controller.
     * @param dt Timestep for prediction.
     */
    void Predict(const InputVector& u, Time dt) {
        // Find discrete A and Q
        StateMatrix discA;
        StateMatrix discQ;
        // TODO: if dt is static then this can be precomputed
        DiscretizeAQ<States>(m_plant->A(), m_contQ, dt, &discA, &discQ);

        m_xHat = m_plant->CalculateX(m_xHat, u, dt);

        // Pₖ₊₁⁻ = APₖ⁻Aᵀ + Q
        m_P = discA * m_P * discA.transpose() + discQ;

        m_dt = dt;
    }

    /**
     * Correct the state estimate x-hat using the measurements in y.
     *
     * @param u Same control input used in the predict step.
     * @param y Measurement vector.
     */
    void Correct(const InputVector& u, const OutputVector& y) {
        Correct(u, y, m_contR);
    }

    /**
     * Correct the state estimate x-hat using the measurements in y.
     *
     * This is useful for when the measurement noise covariances vary.
     *
     * @param u Same control input used in the predict step.
     * @param y Measurement vector.
     * @param R Continuous measurement noise covariance matrix.
     */
    void Correct(const InputVector& u,
                 const OutputVector& y,
                 const Eigen::Matrix<float, Outputs, Outputs>& R) {
        const auto& C = m_plant->C();
        const auto& D = m_plant->D();

        const Eigen::Matrix<float, Outputs, Outputs> discR =
          DiscretizeR<Outputs>(R, m_dt);

        Eigen::Matrix<float, Outputs, Outputs> S =
          C * m_P * C.transpose() + discR;

        // We want to put K = PCᵀS⁻¹ into Ax = b form so we can solve it more
        // efficiently.
        //
        // K = PCᵀS⁻¹
        // KS = PCᵀ
        // (KS)ᵀ = (PCᵀ)ᵀ
        // SᵀKᵀ = CPᵀ
        //
        // The solution of Ax = b can be found via x = A.solve(b).
        //
        // Kᵀ = Sᵀ.solve(CPᵀ)
        // K = (Sᵀ.solve(CPᵀ))ᵀ
        //
        // Drop the transposes on symmetric matrices S and P.
        //
        // K = (S.solve(CP))ᵀ
        Eigen::Matrix<float, States, Outputs> K =
          S.ldlt().solve(C * m_P).transpose();

        // x̂ₖ₊₁⁺ = x̂ₖ₊₁⁻ + K(y − (Cx̂ₖ₊₁⁻ + Duₖ₊₁))
        m_xHat += K * (y - (C * m_xHat + D * u));

        // Pₖ₊₁⁺ = (I−Kₖ₊₁C)Pₖ₊₁⁻(I−Kₖ₊₁C)ᵀ + Kₖ₊₁RKₖ₊₁ᵀ
        // Use Joseph form for numerical stability
        m_P = (StateMatrix::Identity() - K * C) * m_P *
                (StateMatrix::Identity() - K * C).transpose() +
              K * discR * K.transpose();
    }

  private:
    StateVector m_xHat;
    StateMatrix m_P;
    StateMatrix m_contQ;
    Eigen::Matrix<float, Outputs, Outputs> m_contR;
    Time m_dt;

    StateMatrix m_initP;
};

} // namespace lyfast
} // namespace blazing
