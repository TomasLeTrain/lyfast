#pragma once

#include "lyfast/geometry/curve.hpp"
#include "primitives.h"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <vector>

namespace lyfast {
namespace geometry {
class Line : public Curve {
  private:
    Point derivative;
    FLength speed;

  public:
    Point f(float t) override {
        return derivative * t + endpoints[0];
    }

    Point df(float t) override {
        return derivative;
    }

    Point ddf(float t) override {
        return units::origin<FLength>;
    }

    // curvature at point c
    FCurvature c(float t) override {
        return FCurvature(0.0f);
    }

    FCurvature c(float t, Point df) override {
        return FCurvature(0.0f);
    }

    // gets distance at time
    FLength s(float t) override {
        return t * speed;
    }

    float t_by_s(FLength target, float t_guess) override {
        return t_by_s(target);
    }

    // gets time by distance
    float t_by_s(FLength target) override {
        return target / speed;
    }

    Line(std::array<Point, 2> endpoints)
        : Curve(endpoints) {
        derivative = endpoints[1] - endpoints[0];
        speed = derivative.magnitude();

        total_distance = s(1.0);
    }

    ~Line() override = default;
};
} // namespace geometry
} // namespace lyfast
