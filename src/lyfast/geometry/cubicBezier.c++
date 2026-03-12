
#include "lyfast/geometry/cubicBezier.hpp"

namespace blazing {
namespace lyfast {
namespace geometry {

void CubicBezier::compute_coefficient_matrices() {
    const Point& p0 = m_endpoints[0];
    const Point& p1 = controls[0];
    const Point& p2 = controls[1];
    const Point& p3 = m_endpoints[1];

    for (size_t i = 0; i < basis_matrix.size(); i++) {
        Point a0 = (p0 * basis_matrix[i][0]);
        Point a1 = (p1 * basis_matrix[i][1]);
        Point a2 = (p2 * basis_matrix[i][2]);
        Point a3 = (p3 * basis_matrix[i][3]);
        coeff_matrix[i] = a0 + a1 + a2 + a3;
    }
    for (size_t i = 0; i < derivative_basis_matrix.size(); i++) {
        Point a0 = p0 * derivative_basis_matrix[i][0];
        Point a1 = p1 * derivative_basis_matrix[i][1];
        Point a2 = p2 * derivative_basis_matrix[i][2];
        Point a3 = p3 * derivative_basis_matrix[i][3];
        der_coeff_matrix[i] = a0 + a1 + a2 + a3;
    }
    for (size_t i = 0; i < second_der_basis_matrix.size(); i++) {
        Point a0 = p0 * second_der_basis_matrix[i][0];
        Point a1 = p1 * second_der_basis_matrix[i][1];
        Point a2 = p2 * second_der_basis_matrix[i][2];
        Point a3 = p3 * second_der_basis_matrix[i][3];
        second_der_coeff_matrix[i] = a0 + a1 + a2 + a3;
    }
}

Point CubicBezier::f(float t) {
    Point t0 = (coeff_matrix[0] * t * t * t);
    Point t1 = (coeff_matrix[1] * t * t);
    Point t2 = (coeff_matrix[2] * t);
    Point t3 = (coeff_matrix[3]);
    return t0 + t1 + t2 + t3;
}

/**
 * @brief sample derivative of bezier at sample time t
 *
 * @param t time at which to sample
 * @return derivative of bezier at time t
 */
Point CubicBezier::df(float t) {
    Point t0 = (der_coeff_matrix[0] * t * t);
    Point t1 = (der_coeff_matrix[1] * t);
    Point t2 = (der_coeff_matrix[2]);
    return t0 + t1 + t2;
}

Point CubicBezier::ddf(float t) {
    Point t0 = (second_der_coeff_matrix[0] * t);
    Point t1 = (second_der_coeff_matrix[1]);
    return t0 + t1;
}

/**
 * @brief returns speed of bezier at time t
 *
 * @param t time at which to sample
 * @return magnitude of derivative as a Length
 */
FLength CubicBezier::speed(float t) {
    return df(t).magnitude();
}

// curvature at t (Sprunk 12)
FCurvature CubicBezier::c(float t) {
    return c(t, df(t));
}

// curvature at t (Sprunk 12)
FCurvature CubicBezier::c(float t, Point df_t) {
    Point second_derivative = ddf(t);

    // speed function not used to avoid duplicate call to df()
    Exponentiated<Length, std::ratio<3>> speed_cubed =
      units::pow<3>(df_t.magnitude());

    // avoids dividing by zero
    if (speed_cubed.internal() < 1e-6) return 0.0 / Fm;

    return df_t.cross(second_derivative) / speed_cubed;
}

FLength CubicBezier::s(float t) {
    // uses Gaussian quadrature to estimate arc length
    // This implementation is heavily based on vmplib:
    // https://github.com/SerrialError/vmplib/blob/main/src/bezier.cpp

    // clang-format off
		constexpr std::array<float,5> gaussNodes = {-0.9061798459, -0.5384693101, 0.0, 0.5384693101, 0.9061798459};
		static constexpr std::array<float,5> gaussWeights = {0.2369268850, 0.4786286705, 0.5688888889, 0.4786286705, 0.2369268850};
    // clang-format on

    // precomputes new gauss nodes to avoid some small recomputations
    static constexpr std::array<float, 5> new_gaussNodes = [gaussNodes]() {
        std::array<float, 5> result;
        for (int i = 0; i < 5; i++) {
            result[i] = 1.0 + gaussNodes[i];
        }
        return result;
    }();

    const float t2 = t / 2.0f;
    FLength arc_length = FLength(0.0);

    for (size_t i = 0; i < gaussNodes.size(); i++) {
        arc_length += gaussWeights[i] * speed(t2 * new_gaussNodes[i]);
    }
    return t2 * arc_length;
}

// gets time by distance
float CubicBezier::t_by_s(FLength target, float t_guess) {
    // uses Netwon method to find time from arc length
    // This implementation is heavily based on vmplib:
    // https://github.com/SerrialError/vmplib/blob/main/src/bezier.cpp

    constexpr FLength tol = Fin * 1e-2;
    int maxIter = 20;
    int i = 0;
    for (; i < maxIter; i++) {
        FLength f_t = s(t_guess) - target;

        if (units::abs(f_t) < tol) break;

        FLength f_der_t = speed(t_guess);

        t_guess -= f_t / f_der_t;

        if (t_guess < 0) t_guess = 0;
        if (t_guess > 1) t_guess = 1;
    }

    return t_guess;
}

float CubicBezier::t_by_s(FLength target) {
    // uses a starting guess of t = 0.5
    return t_by_s(target, 0.5);
}

CubicBezier::CubicBezier(std::array<Point, 4> controls)
    : CubicBezier(controls[0], controls[1], controls[2], controls[3]) {}

CubicBezier::CubicBezier(Point start, Point control0, Point control1, Point end)
    : Curve(start, end),
      controls({ control0, control1 }) {
    compute_coefficient_matrices();

    m_total_distance = s(1.0);
}
} // namespace geometry
} // namespace lyfast
} // namespace blazing
