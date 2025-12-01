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

    Line(Point start, Point end);
    Line(std::array<Point, 2> endpoints);

    ~Line() override = default;
};
} // namespace geometry
} // namespace lyfast
} // namespace blazing
