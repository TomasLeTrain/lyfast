#include "lyfast/geometry/curve.hpp"

namespace blazing {
namespace lyfast {
namespace geometry {
double Curve::tByClosestPoint(geometry::Point point,
                              double start_t,
                              FLength max_look_dist,
                              FLength resolution) {
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
} // namespace geometry
} // namespace lyfast
} // namespace blazing
