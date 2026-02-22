// Modified from wpilib:
// https://github.com/wpilibsuite/allwpilib/blob/d9eba4bb22797a37d0d9ef9c1807cd073f43b7b3/wpimath/src/main/native/include/frc/DARE.h
// https://github.com/wpilibsuite/allwpilib/blob/d9eba4bb22797a37d0d9ef9c1807cd073f43b7b3/wpimath/src/main/native/include/frc/StateSpaceUtil.h
// https://github.com/wpilibsuite/allwpilib/blob/d9eba4bb22797a37d0d9ef9c1807cd073f43b7b3/wpiutil/src/main/native/include/wpi/Algorithm.h
// https://github.com/wpilibsuite/allwpilib/blob/d9eba4bb22797a37d0d9ef9c1807cd073f43b7b3/wpimath/src/main/native/include/frc/system/Discretization.h
// https://github.com/wpilibsuite/allwpilib/blob/d9eba4bb22797a37d0d9ef9c1807cd073f43b7b3/wpimath/src/main/native/include/frc/controller/LinearQuadraticRegulator.h
//
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "Eigen/Eigenvalues"
#include "Eigen/LU"
#include "units/units.hpp"
#include <expected>
#include <string_view>

namespace blazing {
namespace lyfast {

/**
 * Calls f(i, elem) for each element of elems where i is the index of the
 * element in elems and elem is the element.
 *
 * @param f The callback.
 * @param elems The elements.
 */
template<typename F, typename... Ts>
constexpr void for_each(F&& f, Ts&&... elems) {
    [&]<size_t... Is>(std::index_sequence<Is...>) {
        (f(Is, elems), ...);
    }(std::index_sequence_for<Ts...> {});
}

/**
 * Creates a cost matrix from the given vector for use with LQR.
 *
 * The cost matrix is constructed using Bryson's rule. The inverse square of
 * each tolerance is placed on the cost matrix diagonal. If a tolerance is
 * infinity, its cost matrix entry is set to zero.
 *
 * @param tolerances An array. For a Q matrix, its elements are the maximum
 *                   allowed excursions of the states from the reference. For an
 *                   R matrix, its elements are the maximum allowed excursions
 *                   of the control inputs from no actuation.
 * @return State excursion or control effort cost matrix.
 */
template<std::same_as<float>... Ts>
constexpr Eigen::Matrix<float, sizeof...(Ts), sizeof...(Ts)>
MakeCostMatrix(Ts... tolerances) {
    Eigen::Matrix<float, sizeof...(Ts), sizeof...(Ts)> result;

    for (int row = 0; row < result.rows(); ++row) {
        for (int col = 0; col < result.cols(); ++col) {
            if (row != col) {
                result(row, col) = 0.0;
            }
        }
    }

    std::for_each(
      [&](int i, float tolerance) {
          if (tolerance == std::numeric_limits<float>::infinity()) {
              result(i, i) = 0.0;
          } else {
              result(i, i) = 1.0 / (tolerance * tolerance);
          }
      },
      tolerances...);

    return result;
}

/**
 * Creates a cost matrix from the given vector for use with LQR.
 *
 * The cost matrix is constructed using Bryson's rule. The inverse square of
 * each element in the input is placed on the cost matrix diagonal. If a
 * tolerance is infinity, its cost matrix entry is set to zero.
 *
 * @param costs An array. For a Q matrix, its elements are the maximum allowed
 *              excursions of the states from the reference. For an R matrix,
 *              its elements are the maximum allowed excursions of the control
 *              inputs from no actuation.
 * @return State excursion or control effort cost matrix.
 */
template<size_t N>
constexpr Eigen::Matrix<float, N, N>
MakeCostMatrix(const std::array<float, N>& costs) {
    Eigen::Matrix<float, N, N> result;

    for (int row = 0; row < result.rows(); ++row) {
        for (int col = 0; col < result.cols(); ++col) {
            if (row == col) {
                if (costs[row] == std::numeric_limits<float>::infinity()) {
                    result(row, col) = 0.0;
                } else {
                    result(row, col) = 1.0 / (costs[row] * costs[row]);
                }
            } else {
                result(row, col) = 0.0;
            }
        }
    }

    return result;
}

/**
 * Returns true if (A, B) is a stabilizable pair.
 *
 * (A, B) is stabilizable if and only if the uncontrollable eigenvalues of A, if
 * any, have absolute values less than one, where an eigenvalue is
 * uncontrollable if rank([λI - A, B]) < n where n is the number of states.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param A System matrix.
 * @param B Input matrix.
 */
template<int States, int Inputs>
bool IsStabilizable(const Eigen::Matrix<float, States, States>& A,
                    const Eigen::Matrix<float, States, Inputs>& B) {
    Eigen::EigenSolver<Eigen::Matrix<float, States, States>> es { A, false };

    for (int i = 0; i < A.rows(); ++i) {
        if (std::norm(es.eigenvalues()[i]) < 1) {
            continue;
        }

        if constexpr (States != Eigen::Dynamic && Inputs != Eigen::Dynamic) {
            Eigen::Matrix<std::complex<float>, States, States + Inputs> E;
            E << es.eigenvalues()[i] * Eigen::Matrix<std::complex<float>,
                                                     States,
                                                     States>::Identity() -
                   A,
              B;

            Eigen::ColPivHouseholderQR<
              Eigen::Matrix<std::complex<float>, States, States + Inputs>>
              qr { E };
            if (qr.rank() < States) {
                return false;
            }
        } else {
            Eigen::MatrixXcd E { A.rows(), A.rows() + B.cols() };
            E << es.eigenvalues()[i] *
                     Eigen::MatrixXcd::Identity(A.rows(), A.rows()) -
                   A,
              B;

            Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qr { E };
            if (qr.rank() < A.rows()) {
                return false;
            }
        }
    }
    return true;
}

/**
 * Returns true if (A, C) is a detectable pair.
 *
 * (A, C) is detectable if and only if the unobservable eigenvalues of A, if
 * any, have absolute values less than one, where an eigenvalue is unobservable
 * if rank([λI - A; C]) < n where n is the number of states.
 *
 * @tparam States Number of states.
 * @tparam Outputs Number of outputs.
 * @param A System matrix.
 * @param C Output matrix.
 */
template<int States, int Outputs>
bool IsDetectable(const Eigen::Matrix<float, States, States>& A,
                  const Eigen::Matrix<float, Outputs, States>& C) {
    return IsStabilizable<States, Outputs>(A.transpose(), C.transpose());
}

/**
 * Clamps input vector between system's minimum and maximum allowable input.
 *
 * @tparam Inputs Number of inputs.
 * @param u Input vector to clamp.
 * @param umin The minimum input magnitude.
 * @param umax The maximum input magnitude.
 * @return Clamped input vector.
 */
template<int Inputs>
constexpr Eigen::Vector<float, Inputs>
ClampInputMaxMagnitude(const Eigen::Vector<float, Inputs>& u,
                       const Eigen::Vector<float, Inputs>& umin,
                       const Eigen::Vector<float, Inputs>& umax) {
    Eigen::Vector<float, Inputs> result;
    for (int i = 0; i < u.rows(); ++i) {
        result(i) = std::clamp(u(i), umin(i), umax(i));
    }
    return result;
}

/**
 * Renormalize all inputs if any exceeds the maximum magnitude. Useful for
 * systems such as differential drivetrains.
 *
 * @tparam Inputs      Number of inputs.
 * @param u            The input vector.
 * @param maxMagnitude The maximum magnitude any input can have.
 * @return The normalizedInput
 */
template<int Inputs>
Eigen::Vector<float, Inputs>
DesaturateInputVector(const Eigen::Vector<float, Inputs>& u,
                      float maxMagnitude) {
    float maxValue = u.template lpNorm<Eigen::Infinity>();

    if (maxValue > maxMagnitude) {
        return u * maxMagnitude / maxValue;
    }
    return u;
}

/**
 * Discretizes the given continuous A and B matrices.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param contA Continuous system matrix.
 * @param contB Continuous input matrix.
 * @param dt    Discretization timestep.
 * @param discA Storage for discrete system matrix.
 * @param discB Storage for discrete input matrix.
 */
template<int States, int Inputs>
void DiscretizeAB(const Eigen::Matrix<float, States, States>& contA,
                  const Eigen::Matrix<float, States, Inputs>& contB,
                  Time dt,
                  Eigen::Matrix<float, States, States>* discA,
                  Eigen::Matrix<float, States, Inputs>* discB) {
    // M = [A  B]
    //     [0  0]
    Eigen::Matrix<float, States + Inputs, States + Inputs> M;
    M.template block<States, States>(0, 0) = contA;
    M.template block<States, Inputs>(0, States) = contB;
    M.template block<Inputs, States + Inputs>(States, 0).setZero();

    // ϕ = eᴹᵀ = [A_d  B_d]
    //           [ 0    I ]
    Eigen::Matrix<float, States + Inputs, States + Inputs> phi =
      (M * dt.internal()).exp();

    *discA = phi.template block<States, States>(0, 0);
    *discB = phi.template block<States, Inputs>(0, States);
}

/**
 * Errors the DARE solver can encounter.
 */
enum class DAREError {
    /// Q was not symmetric.
    QNotSymmetric,
    /// Q was not positive semidefinite.
    QNotPositiveSemidefinite,
    /// R was not symmetric.
    RNotSymmetric,
    /// R was not positive definite.
    RNotPositiveDefinite,
    /// (A, B) pair was not stabilizable.
    ABNotStabilizable,
    /// (A, C) pair where Q = CᵀC was not detectable.
    ACNotDetectable,
};

/**
 * Converts the given DAREError enum to a string.
 */
constexpr std::string_view to_string(const DAREError& error) {
    switch (error) {
        case DAREError::QNotSymmetric: return "Q was not symmetric.";
        case DAREError::QNotPositiveSemidefinite:
            return "Q was not positive semidefinite.";
        case DAREError::RNotSymmetric: return "R was not symmetric.";
        case DAREError::RNotPositiveDefinite:
            return "R was not positive definite.";
        case DAREError::ABNotStabilizable:
            return "(A, B) pair was not stabilizable.";
        case DAREError::ACNotDetectable:
            return "(A, C) pair where Q = CᵀC was not detectable.";
    }

    return "";
}

namespace detail {

/**
 * Computes the unique stabilizing solution X to the discrete-time algebraic
 * Riccati equation:
 *
 *   AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
 *
 * This internal function skips expensive precondition checks for increased
 * performance. The solver may hang if any of the following occur:
 * <ul>
 *   <li>Q isn't symmetric positive semidefinite</li>
 *   <li>R isn't symmetric positive definite</li>
 *   <li>The (A, B) pair isn't stabilizable</li>
 *   <li>The (A, C) pair where Q = CᵀC isn't detectable</li>
 * </ul>
 * Only use this function if you're sure the preconditions are met.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param A The system matrix.
 * @param B The input matrix.
 * @param Q The state cost matrix.
 * @param R_llt The LLT decomposition of the input cost matrix.
 * @return Solution to the DARE.
 */
template<int States, int Inputs>
Eigen::Matrix<float, States, States>
DARE(const Eigen::Matrix<float, States, States>& A,
     const Eigen::Matrix<float, States, Inputs>& B,
     const Eigen::Matrix<float, States, States>& Q,
     const Eigen::LLT<Eigen::Matrix<float, Inputs, Inputs>>& R_llt) {
    using StateMatrix = Eigen::Matrix<float, States, States>;

    // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
    //
    // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang "Structure-Preserving
    //     Algorithms for Periodic Discrete-Time Algebraic Riccati Equations",
    //     International Journal of Control, 77:8, 767-788, 2004.
    //     DOI: 10.1080/00207170410001714988

    // A₀ = A
    // G₀ = BR⁻¹Bᵀ
    // H₀ = Q
    StateMatrix A_k = A;
    StateMatrix G_k = B * R_llt.solve(B.transpose());
    StateMatrix H_k;
    StateMatrix H_k1 = Q;

    do {
        H_k = H_k1;

        // W = I + GₖHₖ
        StateMatrix W =
          StateMatrix::Identity(H_k.rows(), H_k.cols()) + G_k * H_k;

        auto W_solver = W.lu();

        // Solve WV₁ = Aₖ for V₁
        StateMatrix V_1 = W_solver.solve(A_k);

        // Solve V₂Wᵀ = Gₖ for V₂
        //
        // We want to put V₂Wᵀ = Gₖ into Ax = b form so we can solve it more
        // efficiently.
        //
        // V₂Wᵀ = Gₖ
        // (V₂Wᵀ)ᵀ = Gₖᵀ
        // WV₂ᵀ = Gₖᵀ
        //
        // The solution of Ax = b can be found via x = A.solve(b).
        //
        // V₂ᵀ = W.solve(Gₖᵀ)
        // V₂ = W.solve(Gₖᵀ)ᵀ
        //
        // Since W, Gₖ, and Hₖ are symmetric, drop the transposes on Gₖ and V₂.
        //
        // V₂ = W.solve(Gₖ)
        StateMatrix V_2 = W_solver.solve(G_k);

        // Gₖ₊₁ = Gₖ + AₖV₂Aₖᵀ
        // Hₖ₊₁ = Hₖ + V₁ᵀHₖAₖ
        // Aₖ₊₁ = AₖV₁
        G_k += A_k * V_2 * A_k.transpose();
        H_k1 = H_k + V_1.transpose() * H_k * A_k;
        A_k *= V_1;

        // while |Hₖ₊₁ − Hₖ| > ε |Hₖ₊₁|
    } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

    return H_k1;
}

} // namespace detail

/**
 * Computes the unique stabilizing solution X to the discrete-time algebraic
 * Riccati equation:
 *
 *   AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param A The system matrix.
 * @param B The input matrix.
 * @param Q The state cost matrix.
 * @param R The input cost matrix.
 * @param checkPreconditions Whether to check preconditions (30% less time if
 *   user is sure precondtions are already met).
 * @return Solution to the DARE on success, or DAREError on failure.
 */
template<int States, int Inputs>
std::expected<Eigen::Matrix<float, States, States>, DAREError>
DARE(const Eigen::Matrix<float, States, States>& A,
     const Eigen::Matrix<float, States, Inputs>& B,
     const Eigen::Matrix<float, States, States>& Q,
     const Eigen::Matrix<float, Inputs, Inputs>& R,
     bool checkPreconditions = true) {
    if (checkPreconditions) {
        // Require R be symmetric
        if ((R - R.transpose()).norm() > 1e-10) {
            return std::unexpected { DAREError::RNotSymmetric };
        }
    }

    // Require R be positive definite
    auto R_llt = R.llt();
    if (R_llt.info() != Eigen::Success) {
        return std::unexpected { DAREError::RNotPositiveDefinite };
    }

    if (checkPreconditions) {
        // Require Q be symmetric
        if ((Q - Q.transpose()).norm() > 1e-10) {
            return std::unexpected { DAREError::QNotSymmetric };
        }

        // Require Q be positive semidefinite
        //
        // If Q is a symmetric matrix with a decomposition LDLᵀ, the number of
        // positive, negative, and zero diagonal entries in D equals the number
        // of positive, negative, and zero eigenvalues respectively in Q (see
        // https://en.wikipedia.org/wiki/Sylvester's_law_of_inertia).
        //
        // Therefore, D having no negative diagonal entries is sufficient to
        // prove Q is positive semidefinite.
        auto Q_ldlt = Q.ldlt();
        if (Q_ldlt.info() != Eigen::Success ||
            (Q_ldlt.vectorD().array() < 0.0).any()) {
            return std::unexpected { DAREError::QNotPositiveSemidefinite };
        }

        // Require (A, B) pair be stabilizable
        if (!IsStabilizable<States, Inputs>(A, B)) {
            return std::unexpected { DAREError::ABNotStabilizable };
        }

        // Require (A, C) pair be detectable where Q = CᵀC
        //
        // Q = CᵀC = PᵀLDLᵀP
        // C = √(D)LᵀP
        Eigen::Matrix<float, States, States> C =
          Q_ldlt.vectorD().cwiseSqrt().asDiagonal() *
          Eigen::Matrix<float, States, States> {
              Q_ldlt.matrixL().transpose()
          } *
          Q_ldlt.transpositionsP();

        if (!IsDetectable<States, States>(A, C)) {
            return std::unexpected { DAREError::ACNotDetectable };
        }
    }

    return detail::DARE<States, Inputs>(A, B, Q, R_llt);
}

/**
Computes the unique stabilizing solution X to the discrete-time algebraic
Riccati equation:

  AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0

This is equivalent to solving the original DARE:

  A₂ᵀXA₂ − X − A₂ᵀXB(BᵀXB + R)⁻¹BᵀXA₂ + Q₂ = 0

where A₂ and Q₂ are a change of variables:

  A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ

This overload of the DARE is useful for finding the control law uₖ that
minimizes the following cost function subject to xₖ₊₁ = Axₖ + Buₖ.

@verbatim
    ∞ [xₖ]ᵀ[Q  N][xₖ]
J = Σ [uₖ] [Nᵀ R][uₖ] ΔT
   k=0
@endverbatim

This is a more general form of the following. The linear-quadratic regulator
is the feedback control law uₖ that minimizes the following cost function
subject to xₖ₊₁ = Axₖ + Buₖ:

@verbatim
    ∞
J = Σ (xₖᵀQxₖ + uₖᵀRuₖ) ΔT
   k=0
@endverbatim

This can be refactored as:

@verbatim
    ∞ [xₖ]ᵀ[Q 0][xₖ]
J = Σ [uₖ] [0 R][uₖ] ΔT
   k=0
@endverbatim

@tparam States Number of states.
@tparam Inputs Number of inputs.
@param A The system matrix.
@param B The input matrix.
@param Q The state cost matrix.
@param R The input cost matrix.
@param N The state-input cross cost matrix.
@param checkPreconditions Whether to check preconditions (30% less time if user
  is sure precondtions are already met).
@return Solution to the DARE on success, or DAREError on failure.
*/
template<int States, int Inputs>
std::expected<Eigen::Matrix<float, States, States>, DAREError>
DARE(const Eigen::Matrix<float, States, States>& A,
     const Eigen::Matrix<float, States, Inputs>& B,
     const Eigen::Matrix<float, States, States>& Q,
     const Eigen::Matrix<float, Inputs, Inputs>& R,
     const Eigen::Matrix<float, States, Inputs>& N,
     bool checkPreconditions = true) {
    if (checkPreconditions) {
        // Require R be symmetric
        if ((R - R.transpose()).norm() > 1e-10) {
            return std::unexpected { DAREError::RNotSymmetric };
        }
    }

    // Require R be positive definite
    auto R_llt = R.llt();
    if (R_llt.info() != Eigen::Success) {
        return std::unexpected { DAREError::RNotPositiveDefinite };
    }

    // This is a change of variables to make the DARE that includes Q, R, and N
    // cost matrices fit the form of the DARE that includes only Q and R cost
    // matrices.
    //
    // This is equivalent to solving the original DARE:
    //
    //   A₂ᵀXA₂ − X − A₂ᵀXB(BᵀXB + R)⁻¹BᵀXA₂ + Q₂ = 0
    //
    // where A₂ and Q₂ are a change of variables:
    //
    //   A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ
    Eigen::Matrix<float, States, States> A_2 =
      A - B * R_llt.solve(N.transpose());
    Eigen::Matrix<float, States, States> Q_2 =
      Q - N * R_llt.solve(N.transpose());

    if (checkPreconditions) {
        // Require Q be symmetric
        if ((Q_2 - Q_2.transpose()).norm() > 1e-10) {
            return std::unexpected { DAREError::QNotSymmetric };
        }

        // Require Q be positive semidefinite
        //
        // If Q is a symmetric matrix with a decomposition LDLᵀ, the number of
        // positive, negative, and zero diagonal entries in D equals the number
        // of positive, negative, and zero eigenvalues respectively in Q (see
        // https://en.wikipedia.org/wiki/Sylvester's_law_of_inertia).
        //
        // Therefore, D having no negative diagonal entries is sufficient to
        // prove Q is positive semidefinite.
        auto Q_ldlt = Q_2.ldlt();
        if (Q_ldlt.info() != Eigen::Success ||
            (Q_ldlt.vectorD().array() < 0.0).any()) {
            return std::unexpected { DAREError::QNotPositiveSemidefinite };
        }

        // Require (A, B) pair be stabilizable
        if (!IsStabilizable<States, Inputs>(A_2, B)) {
            return std::unexpected { DAREError::ABNotStabilizable };
        }

        // Require (A, C) pair be detectable where Q = CᵀC
        //
        // Q = CᵀC = PᵀLDLᵀP
        // C = √(D)LᵀP
        Eigen::Matrix<float, States, States> C =
          Q_ldlt.vectorD().cwiseSqrt().asDiagonal() *
          Eigen::Matrix<float, States, States> {
              Q_ldlt.matrixL().transpose()
          } *
          Q_ldlt.transpositionsP();

        if (!IsDetectable<States, States>(A_2, C)) {
            return std::unexpected { DAREError::ACNotDetectable };
        }
    }

    return detail::DARE<States, Inputs>(A_2, B, Q_2, R_llt);
}

/**
 * Constructs a controller with the given coefficients and plant.
 *
 * @param A  Continuous system matrix of the plant being controlled.
 * @param B  Continuous input matrix of the plant being controlled.
 * @param Q  The state cost matrix.
 * @param R  The input cost matrix.
 * @param N  The state-input cross-term cost matrix.
 * @param dt Discretization timestep.
 * @throws std::invalid_argument If the system is unstabilizable.
@return Solution to the DARE on success, or DAREError on failure.
 */
template<int States, int Inputs>
std::expected<Eigen::Matrix<float, Inputs, States>, DAREError>
LinearQuadraticRegulator_K(const Eigen::Matrix<float, States, States>& A,
                           const Eigen::Matrix<float, States, Inputs>& B,
                           const Eigen::Matrix<float, States, States>& Q,
                           const Eigen::Matrix<float, Inputs, Inputs>& R,
                           const Eigen::Matrix<float, States, Inputs>& N,
                           Time dt) {
    Eigen::Matrix<float, States, States> discA;
    Eigen::Matrix<float, States, Inputs> discB;
    DiscretizeAB<States, Inputs>(A, B, dt, &discA, &discB);

    if (auto S = DARE<States, Inputs>(discA, discB, Q, R, N)) {
        // K = (BᵀSB + R)⁻¹(BᵀSA + Nᵀ)
        return (discB.transpose() * S.value() * discB + R)
          .llt()
          .solve(discB.transpose() * S.value() * discA + N.transpose());
    } else {
        return std::unexpected { S.error() };
    }
}

} // namespace lyfast
} // namespace blazing
