#pragma once

#include "lyfast/geometry/curve.hpp"
#include "lyfast/geometry/primitives.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <cassert>
#include <vector>

namespace blazing {
namespace lyfast {
namespace geometry {
class Spline : public Curve {
  private:
    float num_curves;
    std::vector<Curve*> m_curves;

    // stores the sum of arc lengths up to and before the i'th curve
    std::vector<FLength> distance_to_curve = { 0_m };

    // transfoms time t for the whole curve into a time u for the innner curve
    std::pair<size_t, float> t_to_u(float t);

  public:
    Point f(float t) override;

    Point df(float t) override;
    Point ddf(float t) override;

    FCurvature c(float t) override;
    FCurvature c(float t, Point df) override;

    FLength s(float t) override;

    float t_by_s(FLength target, float t_guess) override;
    float t_by_s(FLength target) override;

    double findClosestPointT(Point point) override {
        Length best = Length(INFINITY);
        double result = 0;

        double samples = 800;

        for (double t = 0; t <= 1; t += 1.0 / samples) {
            Length curr_distance = point.distanceTo(f(t));
            if (curr_distance < best) {
                best = curr_distance;
                result = t;
            }
        }
        return result;
    }

    Spline(std::vector<Curve*>&& curves);

    ~Spline() override = default;
};
} // namespace geometry
} // namespace lyfast
} // namespace blazing
