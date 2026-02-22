#include "lyfast/lqr.hpp"
#include "Eigen/Dense"
#include "blazing/utils.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "unsupported/Eigen/MatrixFunctions"
#include <utility>

namespace blazing {
namespace lyfast {
namespace lqr {

std::pair<Eigen::MatrixXf, Eigen::MatrixXf>
discretizeAB(const Eigen::MatrixXf& contA,
             const Eigen::MatrixXf& contB,
             Time delta_time) {

    int states = contA.rows();
    int inputs = contB.cols();

    // Create an augmented matrix M
    Eigen::MatrixXf M(states + inputs, states + inputs);
    M.setZero();

    // Top-left block is A
    M.topLeftCorner(states, states) = contA;

    // Top-right block is B
    M.topRightCorner(states, inputs) = contB;

    // Matrix exponential of M * dt
    Eigen::MatrixXf phi = (M * delta_time.convert(sec)).exp();

    // Discretized A (A_d) is the top-left block of phi
    Eigen::MatrixXf discA = phi.topLeftCorner(states, states);

    // Discretized B (B_d) is the top-right block of phi
    Eigen::MatrixXf discB = phi.topRightCorner(states, inputs);

    return { discA, discB };
}

Eigen::MatrixXf dareSolver(const Eigen::MatrixXf& A,
                           const Eigen::MatrixXf& B,
                           const Eigen::MatrixXf& Q,
                           const Eigen::MatrixXf& R) {
    Eigen::MatrixXf X = Q;
    Eigen::MatrixXf X_prev;
    Eigen::MatrixXf K, temp;

    for (int i = 0; i < 1000; ++i) {
        // Iterate until convergence
        X_prev = X;
        K = (R + B.transpose() * X * B).inverse() * B.transpose() * X * A;
        X = A.transpose() * X * (A - B * K) + Q;

        if ((X - X_prev).norm() < 1e-6) {
            break;
        }
    }

    return X;
}

blazing::LeftRightSpeeds doStuff() {
    units::V2Position errorXY { 10_in, 1_in };
    Angle errorAngle = 10_stDeg;
    FLinearVelocity desired_vel = 40_inps;

    const Eigen::Vector3f error(errorXY.x.internal(),
                                errorXY.y.internal(),
                                errorAngle.internal());

    const Eigen::Matrix3f Q {
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 }
    };

    const Eigen::Matrix3f R {
        { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 }
    };

    const Eigen::Matrix3f A {
        { 0.0, 0.0, 0.0                    },
        { 0.0, 0.0, desired_vel.internal() },
        { 0.0, 0.0, 0.0                    }
    };
    const Eigen::Matrix<float, 3, 2> B {
        { 1.0, 0.0 },
        { 0.0, 0.0 },
        { 0.0, 1.0 }
    };

    const auto discAB = discretizeAB(A, B, 10_msec);

    const Eigen::MatrixXf X = dareSolver(discAB.first, discAB.second, Q, R);
    const Eigen::MatrixXf K =
      (R + discAB.second.transpose() * X * discAB.second).inverse() *
      discAB.second.transpose() * X * discAB.first;

    const Eigen::Vector2f u = K * error;

    blazing::LeftRightSpeeds speeds = { u.x() * mps, u.y() * mps };
    return speeds;
}

} // namespace lqr
} // namespace lyfast
} // namespace blazing
