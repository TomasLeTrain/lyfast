#pragma once

#include "lyfast/geometry/primitives.hpp"
#include "units/Pose.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <array>
#include <vector>

namespace blazing {
namespace lyfast {
namespace geometry {
class Curve {
  public:
    FLength total_distance;
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

    // old impl to find closest curve point to some point
    virtual double findClosestPointT(Point point) = 0;

    // searches equidistantly through the curve
    // somewhat expensive as it calculates arc length many times
    virtual double tByClosestPoint(geometry::Point point,
                                   double start_t = 0,
                                   FLength max_look_dist = FLength(INFINITY),
                                   FLength resolution = 1_in) {
        FLength best = Length(INFINITY);
        int result = 0;

        FLength original_start_dist = s(start_t);
        // either some max look dist or look until the end of the array
        FLength original_end_dist =
          units::min(original_start_dist + max_look_dist, s(1));

        FLength start_dist = original_start_dist;
        FLength end_dist = original_end_dist;

        for (Length curr_dist = start_dist; curr_dist <= end_dist;
             curr_dist += resolution) {
            // get index by distance
            float curr_t = t_by_s(curr_dist);
            Length curr_distance = point.distanceTo(f(curr_t));

            if (curr_distance < best) {
                best = curr_distance;
                result = curr_t;
            }
        }

        // here result is likely close to optimal, but we can run second loop to
        // find closest one
        // the clamping makes sure we don't bypass the set constraints
        auto result_dist = s(result);

        start_dist = units::clamp(result_dist - resolution,
                                  original_start_dist,
                                  original_end_dist);
        end_dist = units::clamp(result_dist + resolution,
                                original_start_dist,
                                original_end_dist);

        // search with 10x resolution in small search space around found
        // solution
        for (Length curr_dist = start_dist; curr_dist <= end_dist;
             curr_dist += resolution / 10.0) {
            // get t by distance
            float curr_t = t_by_s(curr_dist);
            Length curr_distance = point.distanceTo(f(curr_t));

            if (curr_distance < best) {
                best = curr_distance;
                result = curr_t;
            }
        }

        // here result is very likely optimal
        return result;
    }

    Curve(Point first_endpoint, Point last_endpoint)
        : endpoints({ first_endpoint, last_endpoint }) {}

    Curve(std::array<Point, 2> endpoints)
        : endpoints(endpoints) {}

    virtual ~Curve() = default;
};
} // namespace geometry
} // namespace lyfast
} // namespace blazing
