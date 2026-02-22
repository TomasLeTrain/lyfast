#include "lyfast/geometry/line.hpp"

namespace blazing {
namespace lyfast {
namespace geometry {

Point Line::f(float t) {
    return derivative * t + endpoints[0];
}

Point Line::df(float t) {
    return derivative;
}

Point Line::ddf(float t) {
    return units::origin<FLength>;
}

// curvature at point c
FCurvature Line::c(float t) {
    return FCurvature(0);
}

FCurvature Line::c(float t, Point df) {
    return FCurvature(0);
}

// gets distance at time
FLength Line::s(float t) {
    return t * speed;
}

float Line::t_by_s(FLength target, float t_guess) {
    return t_by_s(target);
}

// gets time by distance
float Line::t_by_s(FLength target) {
    return target / speed;
}

Line::Line(Point start, Point end)
    : Curve(start, end) {

    derivative = endpoints[1] - endpoints[0];
    speed = derivative.magnitude();

    total_distance = s(1.0);
}

Line::Line(std::array<Point, 2> endpoints)
    : Line(endpoints[0], endpoints[1]) {}

} // namespace geometry
} // namespace lyfast
} // namespace blazing
