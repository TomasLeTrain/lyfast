#pragma once

#include "blazing/utils.hpp"
#include "lyfast/controllers/path_pose_feedback.hpp"
#include "units/Pose.hpp"

namespace blazing {
namespace lyfast {
namespace state_space {

struct LTVUnicycleController {
  public:
    // stores state of the system
    struct State {
        units::Pose pose;
        DifferentialSpeeds velocities;

        // allow convinient convertion
        static State
        fromPathPoseFeedback(PathPoseFeedbackT path_pose_feedback) {
            return State { .pose = path_pose_feedback.pose,
                           .velocities = path_pose_feedback.velocities };
        }
    };

  private:
    State m_state;
    State m_next_reference;
    State m_current_reference;
    Time m_delta_time = 10_msec;
    std::optional<DifferentialSpeeds> m_input = std::nullopt;

    std::array<float, 3> m_Q;
    std::array<float, 2> m_R;
    FTime m_input_delay = 0_msec;

  public:
    void setState(State new_state);
    void setNextReference(State new_reference);
    void setDeltaTime(Time delta_time);
    void setTimeDelay(Time time_delay);

    std::optional<DifferentialSpeeds> getInput();

    // Q matrix determined by bryson's rule
    void setQMatrix(std::array<float, 3> Q);

    // R matrix determined by bryson's rule
    void setRMatrix(std::array<float, 2> R);

    void compute();

    DifferentialSpeeds
    update(PathPoseFeedbackT state, PathPoseFeedbackT reference, Time duration);

    LTVUnicycleController();
    LTVUnicycleController(std::array<float, 3> Q,
                          std::array<float, 2> R,
                          Time input_delay = 0_msec);
};

} // namespace state_space
} // namespace lyfast
} // namespace blazing
