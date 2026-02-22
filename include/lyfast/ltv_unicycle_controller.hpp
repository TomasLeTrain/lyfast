#pragma once

#include "blazing/utils.hpp"
#include "units/Pose.hpp"

namespace blazing {
namespace lyfast {
namespace state_space {

struct LTVUnicycleController {
  public:
    // stores state of the system
    struct State {
        units::Pose pose;
        LinearVelocity linear_velocity;
        AngularVelocity angular_velocity;
    };

  private:
    State m_state;
    State m_reference;
    std::optional<DifferentialSpeeds> m_input = std::nullopt;

    std::array<float, 3> m_Q;
    std::array<float, 2> m_R;

  public:
    void setState(State new_state);
    void setReference(State new_reference);

    std::optional<DifferentialSpeeds> getInput();

    // Q matrix determined by bryson's rule
    void setQMatrix(std::array<float, 3> Q);

    // R matrix determined by bryson's rule
    void setRMatrix(std::array<float, 2> R);

    void update();

    LTVUnicycleController();
    LTVUnicycleController(std::array<float, 3> Q, std::array<float, 2> R);
};

} // namespace state_space
} // namespace lyfast
} // namespace blazing
