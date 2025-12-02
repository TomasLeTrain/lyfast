#pragma once

#include "lyfast/geometry/curve.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <vector>

namespace blazing {
namespace lyfast {
namespace geometry {

class CubicBezier : public Curve {
  private:
    // basis matrices
    const std::array<std::array<float, 4>, 4> basis_matrix {
        { { -1, 3, -3, 1 }, { 3, -6, 3, 0 }, { -3, 3, 0, 0 }, { 1, 0, 0, 0 } }
    };

    const std::array<std::array<float, 4>, 3> derivative_basis_matrix {
        { { -3, 9, -9, 3 }, { 6, -12, 6, 0 }, { -3, 3, 0, 0 } }
    };
    const std::array<std::array<float, 4>, 2> second_der_basis_matrix {
        { { -6, 18, -18, 6 }, { 6, -12, 6, 0 } }
    };

    // coefficient matrices
    std::array<Point, 4> coeff_matrix;
    std::array<Point, 3> der_coeff_matrix;
    std::array<Point, 2> second_der_coeff_matrix;

    std::array<Point, 2> controls;

    void compute_coefficient_matrices();

  public:
    /**
     * @brief sample bezier at sample time t
     *
     * @param t time at which to sample
     * @return bezier point at time t
     */
    Point f(float t) override;
    /**
     * @brief sample derivative of bezier at sample time t
     *
     * @param t time at which to sample
     * @return derivative of bezier at time t
     */
    Point df(float t) override;
    Point ddf(float t) override;
    /**
     * @brief returns speed of bezier at time t
     *
     * @param t time at which to sample
     * @return magnitude of derivative as a Length
     */
    FLength speed(float t);

    // curvature at t (Sprunk 12)
    FCurvature c(float t) override;

    // curvature at t (Sprunk 12)
    FCurvature c(float t, Point df_t) override;

    FLength s(float t) override;

    // gets time by distance
    float t_by_s(FLength target, float t_guess) override;

    float t_by_s(FLength target) override;

    double findClosestPointT(Point point) override {
        Length best = Length(INFINITY);
        double result = 0;

        for (double t = 0; t <= 1; t += 1.0 / 400) {
            Length curr_distance = point.distanceTo(f(t));
            if (curr_distance < best) {
                best = curr_distance;
                result = t;
            }
        }
        return result;
    }

    CubicBezier(std::array<Point, 4> controls);
    CubicBezier(Point start, Point control0, Point control1, Point end);

    ~CubicBezier() override = default;
};
} // namespace geometry
} // namespace lyfast
} // namespace blazing
