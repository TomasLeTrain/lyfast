#pragma once

#include "lyfast/geometry/curve.hpp"
#include "lyfast/geometry/primitives.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <vector>

namespace blazing {
namespace lyfast {
namespace geometry {
class Line : public Curve {
  private:
    Point derivative;
    FLength speed;

  public:
    Point f(float t) override;

    Point df(float t) override;
    Point ddf(float t) override;

    // curvature at point c
    FCurvature c(float t) override;
    FCurvature c(float t, Point df) override;

    // gets distance at time
    FLength s(float t) override;
    float t_by_s(FLength target, float t_guess) override;

    // gets time by distance
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

    Line(Point start, Point end);
    Line(std::array<Point, 2> endpoints);

    ~Line() override = default;
};
} // namespace geometry
} // namespace lyfast
} // namespace blazing
