#include "lyfast/geometry/spline.hpp"

namespace blazing {
namespace lyfast {
namespace geometry {

// transfoms time t for the whole curve into a time u for the innner curve
std::pair<size_t, float> Spline::t_to_u(float t) {
    assert((t <= 1 && t >= 0) && "t is not between 0 and 1");

    // splines are evaluated on the range [0,1) except for the curve's
    // initial and final endpoints
    if (t == 1) {
        // returns the endpoint of the last curve to avoid an out of range
        return { m_curves.size() - 1, 1.0 };
    }

    float fu = t * num_curves;

    return { static_cast<size_t>(fu), std::fmod(fu, 1.0) };
}

Point Spline::f(float t) {
    auto [u, nt] = t_to_u(t);
    return m_curves[u]->f(nt);
}

Point Spline::df(float t) {
    auto [u, nt] = t_to_u(t);

    // have to scale due to change of variables
    return m_curves[u]->df(nt) * num_curves;
}

Point Spline::ddf(float t) {
    auto [u, nt] = t_to_u(t);
    // have to scale due to change of variables
    return m_curves[u]->ddf(nt) * num_curves * num_curves;
}

FCurvature Spline::c(float t) {
    auto [u, nt] = t_to_u(t);
    return m_curves[u]->c(nt);
}

FCurvature Spline::c(float t, Point df) {
    auto [u, nt] = t_to_u(t);

    // since df from global spline time we must convert
    return m_curves[u]->c(nt, df / num_curves);
}

FLength Spline::s(float t) {
    // here we have to handle the past distances as well
    auto [u, nt] = t_to_u(t);
    FLength past_distance = distance_to_curve[u];
    return past_distance + m_curves[u]->s(nt);
}

float Spline::t_by_s(FLength target, float t_guess) {
    // here we have to look up the spline in which the distance is in range

    assert((target <= m_total_distance) &&
           "target is larger than total distance");

    FLength past_distance = 0_Fm;
    size_t current_curve = 0;
    bool found_curve = false;

    for (size_t i = 0; i < m_curves.size(); i++) {
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

    float curve_t;

    if (t_guess < 0.0)
        curve_t = m_curves[current_curve]->t_by_s(target - past_distance);
    else
        curve_t = m_curves[current_curve]->t_by_s(
          target - past_distance,
          // converts t_guess into u time for current spline
          (t_guess * m_curves.size()) - static_cast<float>(current_curve));

    // need to convert the local u time to the [0,1] t
    return (static_cast<float>(current_curve) + curve_t) / m_curves.size();
}

float Spline::t_by_s(FLength target) {
    return t_by_s(target, -1.0);
}

Spline::Spline(std::vector<std::shared_ptr<Curve>> curves)
    : Curve(curves.front()->getFirstEndpoint(),
            curves.back()->getLastEndpoint()),
      m_curves(curves) {
    num_curves = static_cast<float>(m_curves.size());

    // check that endpoints between curves match
    for (size_t i = 1; i < m_curves.size(); i++) {
        assert((
                 // distance from one endpoint to another
                 m_curves[i - 1]->getLastEndpoint().distanceTo(
                   m_curves[i]->getFirstEndpoint())
                 // is bigger than some epsilon
                 < 1_cm) &&
               "curve endpoints do not match");
    }

    Length distance = 0_m;

    // sets the distances to each curve
    for (std::shared_ptr<Curve>& curve : m_curves) {
        distance += curve->getTotalDistance();
        distance_to_curve.emplace_back(distance);
    }

    m_total_distance = distance;
}
} // namespace geometry
} // namespace lyfast
} // namespace blazing
