#pragma once

#include "lyfast/geometry/curve.h"
#include "lyfast/geometry/primitives.h"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <cassert>
#include <vector>

namespace lyfast {
namespace geometry {
class Spline : public Curve {
  private:
    std::vector<Curve> curves;

    // stores the sum of arc lengths up to and before the i'th curve
    std::vector<FLength> distance_to_curve = { 0_m };

    // transfoms time t for the whole curve into a time u for the innner curve
    std::pair<size_t, float> t_to_u(float t) {
        assert((t <= 1 && t >= 0) && "t is not between 0 and 1");

        // splines are evaluated on the range [0,1) except for the curve's
        // initial and final endpoints
        if (t == 1) {
            // returns the endpoint of the last curve to avoid an out of range
            return { this->curves.size() - 1, 1.0 };
        }

        float fu = t * static_cast<float>(this->curves.size());

        return { static_cast<float>(fu), std::fmod(fu, 1.0) };
    }

  public:
    Point f(float t) override {
        auto [u, nt] = this->t_to_u(t);
        return curves[u].f(nt);
    }

    Point df(float t) override {
        auto [u, nt] = this->t_to_u(t);
        return curves[u].df(nt);
    }

    Point ddf(float t) override {
        auto [u, nt] = this->t_to_u(t);
        return curves[u].ddf(nt);
    }

    FCurvature c(float t) override {
        auto [u, nt] = this->t_to_u(t);
        return curves[u].c(nt);
    }

    FLength s(float t) override {
        // here we have to handle the past distances as well
        auto [u, nt] = this->t_to_u(t);
        FLength past_distance = this->distance_to_curve[u];
        return past_distance + this->curves[u].s(nt);
    }

    float t_by_s(FLength target) override {
        // here we have to look up the spline in which the distance is in range

        assert((target <= distance_to_curve.back()) &&
               "target is larger than total distance");

        FLength past_distance = 0_Fm;
        size_t current_curve = 0;
        bool found_curve = false;

        for (size_t i = 0; i < curves.size(); i++) {
            // previous spline always prioritized at endpoints
            if (distance_to_curve[i + 1] >= target) {
                // dist is in this curve
                past_distance = distance_to_curve[i];
                current_curve = i;
                found_curve = true;
                break;
            }
        }

        // no curve found, distance given must be invalid
        assert((found_curve) && "no valid curve found");

        float curve_t = curves[current_curve].t_by_s(target - past_distance);

        // need to convert the local curve_t to the [0,1] t
        return (static_cast<float>(current_curve) + curve_t) /
               this->curves.size();
    }

    Spline(std::vector<Curve> curves)
        : Curve(curves.front().endpoints[0], curves.back().endpoints[1]),
          curves(std::move(curves)) {
        // check that endpoints between curves match
        for (size_t i = 1; i < this->curves.size(); i++) {
            assert((
                     // distance from one endpoint to another
                     this->curves[i - 1].endpoints[1].distanceTo(
                       this->curves[i].endpoints[0])
                     // is bigger than some epsilon
                     < 1_cm) &&
                   "curve endpoints do not match");
        }

        Length distance = 0_m;

        // sets the distances to each curve
        for (Curve& curve : curves) {
            distance += curve.s(1.0);
            distance_to_curve.emplace_back(distance);
        }
    }

    ~Spline() override = default;
};
} // namespace geometry
} // namespace lyfast
