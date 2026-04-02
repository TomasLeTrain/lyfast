#include "lyfast/controllers/ltv_unicycle_controller.hpp"
#include "Eigen/Dense"
#include "blazing/utils.hpp"
#include "lyfast/utils/state_space_utils.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "unsupported/Eigen/MatrixFunctions"
#include <iostream>
#include <utility>

namespace blazing {
namespace lyfast {
namespace state_space {

void LTVUnicycleController::setState(State new_state) {
    m_state = new_state;
}

void LTVUnicycleController::setNextReference(State new_reference) {
    m_current_reference = m_next_reference;
    m_next_reference = new_reference;
}

void LTVUnicycleController::setDeltaTime(Time delta_time) {
    bool delta_has_changed =
      units::abs(m_last_delta_time - delta_time) > 5_msec;

    m_last_delta_time = m_delta_time;
    m_delta_time = delta_time;

    // delta time has changed, need to update the feedback (with new delta time)
    if (delta_has_changed) {
        precomputeforwardsAngleFeedback();
    }
}

void LTVUnicycleController::setTimeDelay(Time input_delay) {
    m_input_delay = input_delay;

    // need to recompute when cost changes
    precomputeforwardsAngleFeedback();
}

void LTVUnicycleController::setMinimumVelocity(
  LinearVelocity minimum_velocity) {
    m_minimum_velocity = minimum_velocity;
}

std::optional<DifferentialSpeeds> LTVUnicycleController::getInput() {
    return m_input;
}

// Q matrix determined by bryson's rule
void LTVUnicycleController::setQMatrix(std::array<float, 3> Q) {
    m_Q = Q;
    // forwardsAngle not dependent on Q
}

// Q matrix determined by bryson's rule
void LTVUnicycleController::setSimpleQMatrix(std::array<float, 2> Q) {
    m_simple_Q = Q;

    // need to recompute when cost changes
    precomputeforwardsAngleFeedback();
}

// R matrix determined by bryson's rule
void LTVUnicycleController::setRMatrix(std::array<float, 2> R) {
    m_R = R;
    // need to recompute when cost changes
    precomputeforwardsAngleFeedback();
}

void LTVUnicycleController::precomputeforwardsAngleFeedback() {
    std::cout << "LQR:computing forwards angle" << std::endl;

    const Eigen::Matrix2f Q = MakeCostMatrix(m_simple_Q); // states x states
    const Eigen::Matrix2f R = MakeCostMatrix(m_R); // inputs x inputs

    // states x states
    const Eigen::Matrix2f A {
        { 0.f, 0.f },
        { 0.f, 0.f }
    };

    // states x inputs
    const Eigen::Matrix2f B {
        { 1.f, 0.f },
        { 0.f, 1.f }
    };

    Eigen::Matrix2f discA;
    Eigen::Matrix2f discB;
    DiscretizeAB<2, 2>(A, B, m_delta_time, &discA, &discB);

    auto R_llt = R.llt();

    // use dare directly for better performance
    auto S = detail::DARE<2, 2>(discA, discB, Q, R_llt);

    // K = (BᵀSB + R)⁻¹(BᵀSA)
    Eigen::Matrix2f K_feedback = (discB.transpose() * S * discB + R)
                                   .llt()
                                   .solve(discB.transpose() * S * discA);

    // apply delay compensation
    if (m_input_delay > 2_msec)
        K_feedback =
          K_feedback *
          (discA - discB * K_feedback).pow(m_input_delay / m_delta_time);

    m_simpler_feedback = { K_feedback(0, 0),
                           K_feedback(0, 1),
                           K_feedback(1, 0),
                           K_feedback(1, 1) };
}

DifferentialSpeeds
LTVUnicycleController::forwardsAngleCompute(units::Pose error_pose) {
    std::cout << "LQR:using forwards angle" << std::endl;
    const Eigen::Vector2f error(error_pose.x.internal(),
                                error_pose.orientation.internal());

    const Eigen::Matrix2f K_feedback {
        { m_simpler_feedback[0], m_simpler_feedback[1] },
        { m_simpler_feedback[2], m_simpler_feedback[3] }
    };

    const Eigen::Vector2f u_feedback = K_feedback * error;

    return { u_feedback.x() * mps, u_feedback.y() * radps };
}

DifferentialSpeeds LTVUnicycleController::fullCompute(units::Pose error_pose) {
    const Eigen::Vector3f error(error_pose.x.internal(),
                                error_pose.y.internal(),
                                error_pose.orientation.internal());

    float A_state_velocity = m_state.velocities.linear_velocity.internal();

    // avoid a dare error by keeping velocity non-zero
    // trigger in case minimum_velocity is low enough for this to trigger
    if (std::abs(A_state_velocity) < 1e-4) {
        A_state_velocity = 1e-4;
    }

    const Eigen::Matrix3f Q = MakeCostMatrix(m_Q); // states x states
    const Eigen::Matrix2f R = MakeCostMatrix(m_R); // inputs x inputs

    // states x states
    const Eigen::Matrix3f A {
        { 0.0, 0.0, 0.0              },
        { 0.0, 0.0, A_state_velocity },
        { 0.0, 0.0, 0.0              }
    };

    // states x inputs
    const Eigen::Matrix<float, 3, 2> B {
        { 1.0, .0  },
        { .0,  .0  },
        { .0,  1.0 }
    };

    Eigen::Matrix<float, 3, 3> discA;
    Eigen::Matrix<float, 3, 2> discB;
    DiscretizeAB<3, 2>(A, B, m_delta_time, &discA, &discB);

    auto R_llt = R.llt();

    // use dare directly for better performance
    auto S = detail::DARE<3, 2>(discA, discB, Q, R_llt);

    // K = (BᵀSB + R)⁻¹(BᵀSA)
    Eigen::Matrix<float, 2, 3> K_feedback =
      (discB.transpose() * S * discB + R)
        .llt()
        .solve(discB.transpose() * S * discA);

    // apply delay compensation
    if (m_input_delay > 2_msec)
        K_feedback =
          K_feedback *
          (discA - discB * K_feedback).pow(m_input_delay / m_delta_time);

    const Eigen::Vector2f u_feedback = K_feedback * error;

    return { u_feedback.x() * mps, u_feedback.y() * radps };
}

// TODO: K only depends on reference velocity - could precompute K for all
// possible velocities for O(1) lookup
// however would also have to recompute all values if changing Q or R
void LTVUnicycleController::compute() {
    const units::V2FPosition local_error =
      (m_next_reference.pose - m_state.pose)
        .rotatedBy(-m_state.pose.orientation);
    const Angle angle_error =
      angleError(m_next_reference.pose.orientation, m_state.pose.orientation);

    const units::Pose error_pose(local_error, angle_error);

    DifferentialSpeeds feedback_velocities;

    // velocity low enough that sideways correction is not possible, just focus
    // on forwards and sideways
    if (units::abs(m_state.velocities.linear_velocity) < m_minimum_velocity) {
        feedback_velocities = forwardsAngleCompute(error_pose);
    } else {
        feedback_velocities = fullCompute(error_pose);
    }

    // feedforward + feedback
    m_input = m_next_reference.velocities + feedback_velocities;
}

DifferentialSpeeds LTVUnicycleController::update(PathPoseFeedbackT state,
                                                 PathPoseFeedbackT reference,
                                                 Time duration) {
    setState(State::fromPathPoseFeedback(state));
    setNextReference(State::fromPathPoseFeedback(reference));
    setDeltaTime(duration);

    compute();
    auto result = getInput();

    if (result.has_value()) {
        return result.value();
    } else {
        // compute would have already logged error
        return { 0_inps, 0_radps };
    }
}

LTVUnicycleController::LTVUnicycleController() {}

LTVUnicycleController::LTVUnicycleController(std::array<float, 3> Q,
                                             std::array<float, 2> simple_Q,
                                             std::array<float, 2> R,
                                             Time input_delay,
                                             LinearVelocity minimum_velocity)
    : m_Q(Q),
      m_simple_Q(simple_Q),
      m_R(R),
      m_input_delay(input_delay),
      m_minimum_velocity(minimum_velocity) {
    precomputeforwardsAngleFeedback();
}

LTVUnicycleController::LTVUnicycleController(std::array<float, 3> Q,
                                             std::array<float, 2> R,
                                             Time input_delay,
                                             LinearVelocity minimum_velocity)
    : LTVUnicycleController(Q,
                            { Q[0], Q[2] },
                            R,
                            input_delay,
                            minimum_velocity) {}
} // namespace state_space
} // namespace lyfast
} // namespace blazing
