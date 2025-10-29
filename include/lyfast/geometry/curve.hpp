#pragma once

#include "lyfast/geometry/primitives.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <vector>

namespace lyfast {
namespace geometry {
class Curve {
  public:
    std::array<Point, 2> endpoints;

    /**
     * @brief sample point of curve at sample time t
     *
     * @param t time at which to sample
     * @return point of curve at time t
     */
    virtual Point f(float t) = 0;

    /**
     * @brief sample first gradient(derivative) of curve at sample time t
     *
     * @param t time at which to sample
     * @return second gradient(derivative) of curve at time t
     */
    virtual Point df(float t) = 0;

    /**
     * @brief sample second gradient(derivative) of curve at sample time t
     *
     * @param t time at which to sample
     * @return second gradient(derivative) of curve at time t
     */
    virtual Point ddf(float t) = 0;

    // curvature at point c
    virtual FCurvature c(float t) = 0;

    // curvature at point c. allows using an already computed value of df
    virtual FCurvature c(float t, Point df) = 0;

    // gets distance at time
    virtual FLength s(float t) = 0;

    // gets time from arc length
    // allows passing in an initial guess of t,
    // useful if a t is known for which s(t) is close to the target
    virtual float t_by_s(FLength target, float t_guess) = 0;

    // gets time by distance
    virtual float t_by_s(FLength target) = 0;

    Curve(Point first_endpoint, Point last_endpoint)
        : endpoints({ first_endpoint, last_endpoint }) {}

    Curve(std::array<Point, 2> endpoints)
        : endpoints(endpoints) {}

    virtual ~Curve() = default;
};
} // namespace geometry
} // namespace lyfast
