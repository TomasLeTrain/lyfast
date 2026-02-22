#include "lyfast/ltv_unicycle_controller.hpp"
#include "Eigen/Dense"
#include "blazing/utils.hpp"
#include "lyfast/state_space_utils.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "unsupported/Eigen/MatrixFunctions"
#include <utility>

namespace blazing {
namespace lyfast {
namespace state_space {

void LTVUnicycleController::setState(State new_state) {
    m_state = new_state;
}

void LTVUnicycleController::setReference(State new_reference) {
    m_reference = new_reference;
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

void LTVUnicycleController::update() {
    const auto local_error =
      (m_reference.pose - m_state.pose).rotatedBy(-m_state.pose.orientation);

    const Angle angle_error =
      angleError(m_reference.pose.orientation, m_state.pose.orientation);

    const Eigen::Vector3f error(local_error.x.internal(),
                                local_error.y.internal(),
                                angle_error.internal());

    // states x states
    const Eigen::Matrix3f Q = MakeCostMatrix(m_Q);

    // inputs x inputs
    const Eigen::Matrix2f R = MakeCostMatrix(m_R);

    // states x inputs
    // TODO: how to use??
    const Eigen::Matrix<float, 3, 2> N {
        {
         0.0, 0.0,
         },
        {
         0.0,   0.0,
         },
        {
         0.0, 0.0,
         }
    };

    float A_reference_velocity = m_state.linear_velocity.internal();

    // avoid lqr error
    if (std::abs(A_reference_velocity) < 1e-6) {
        A_reference_velocity = 1e-6;
    }

    // states x states
    const Eigen::Matrix3f A {
        { 0.0, 0.0, 0.0                  },
        { 0.0, 0.0, A_reference_velocity },
        { 0.0, 0.0, 0.0                  }
    };

    // states x inputs
    const Eigen::Matrix<float, 3, 2> B {
        { 1.0, 0.0 },
        { 0.0, 0.0 },
        { 0.0, 1.0 }
    };

    Eigen::Matrix3f discA;
    Eigen::Matrix<float, 3, 2> discB;

    if (const auto K =
          LinearQuadraticRegulator_K<3, 2>(A, B, Q, R, N, 10_msec)) {
        const Eigen::Vector2f u = K.value() * error;

        m_input = { m_reference.linear_velocity + u.x() * mps,
                    m_reference.angular_velocity + u.y() * radps };
    } else {
        std::cout << "LQR returned error: " << to_string(K.error())
                  << std::endl;
        m_input = std::nullopt;
    }
}

LTVUnicycleController::LTVUnicycleController() {}

LTVUnicycleController::LTVUnicycleController(std::array<float, 3> Q,
                                             std::array<float, 2> R)
    : m_Q(Q),
      m_R(R) {}

} // namespace state_space
} // namespace lyfast
} // namespace blazing
