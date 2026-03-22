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
    m_delta_time = delta_time;
}

void LTVUnicycleController::setTimeDelay(Time input_delay) {
    m_input_delay = input_delay;
}

std::optional<DifferentialSpeeds> LTVUnicycleController::getInput() {
    return m_input;
}

// Q matrix determined by bryson's rule
void LTVUnicycleController::setQMatrix(std::array<float, 3> Q) {
    m_Q = Q;
}

// R matrix determined by bryson's rule
void LTVUnicycleController::setRMatrix(std::array<float, 2> R) {
    m_R = R;
}

// TODO: K only depends on reference velocity - could precompute K for all
// possible velocities for O(1) lookup
// however would also have to recompute all values if changing Q or R
void LTVUnicycleController::compute() {
    const auto local_error = (m_next_reference.pose - m_state.pose)
                               .rotatedBy(-m_state.pose.orientation);

    const Angle angle_error =
      angleError(m_next_reference.pose.orientation, m_state.pose.orientation);

    const Eigen::Vector3f error(local_error.x.internal(),
                                local_error.y.internal(),
                                angle_error.internal());

    const Eigen::Matrix3f Q = MakeCostMatrix(m_Q); // states x states
    const Eigen::Matrix2f R = MakeCostMatrix(m_R); // inputs x inputs

    float A_state_velocity = m_state.velocities.linear_velocity.internal();

    // avoid a dare error by keeping velocity non-zero
    if (std::abs(A_state_velocity) < 1e-4) {
        A_state_velocity = 1e-4;
    }

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
    // auto K_feedback = (discB.transpose() * S * discB + R)
    //                     .llt()
    //                     .solve(discB.transpose() * S * discA);
    //
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

    DifferentialSpeeds feedback_velocities { u_feedback.x() * mps,
                                             u_feedback.y() * radps };

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
                                             std::array<float, 2> R,
                                             Time input_delay)
    : m_Q(Q),
      m_R(R),
      m_input_delay(input_delay) {}

} // namespace state_space
} // namespace lyfast
} // namespace blazing
