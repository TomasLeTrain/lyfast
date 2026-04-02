#pragma once

#include "blazing/utils.hpp"
#include "lyfast/controllers/path_pose_feedback.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"

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

    // used to determine if delta time has changed
    Time m_last_delta_time = -1_sec;
    Time m_delta_time = 20_msec;

    std::optional<DifferentialSpeeds> m_input = std::nullopt;
    std::array<float, 4> m_simpler_feedback;

    std::array<float, 3> m_Q;
    std::array<float, 2> m_simple_Q;
    std::array<float, 2> m_R;

    FTime m_input_delay = 0_msec;
    LinearVelocity m_minimum_velocity = 0_mps;

  public:
    void setState(State new_state);
    void setNextReference(State new_reference);
    void setDeltaTime(Time delta_time);
    void setTimeDelay(Time time_delay);
    void setMinimumVelocity(LinearVelocity minimum_velocity);

    std::optional<DifferentialSpeeds> getInput();

    // Q matrix determined by bryson's rule
    void setQMatrix(std::array<float, 3> Q);

    // [x, theta] -> Q matrix determined by bryson's rule
    void setSimpleQMatrix(std::array<float, 2> Q);

    // R matrix determined by bryson's rule
    void setRMatrix(std::array<float, 2> R);

    void precomputeforwardsAngleFeedback();

    DifferentialSpeeds forwardsAngleCompute(units::Pose error_pose);
    DifferentialSpeeds fullCompute(units::Pose error_pose);

    void compute();

    DifferentialSpeeds
    update(PathPoseFeedbackT state, PathPoseFeedbackT reference, Time duration);

    LTVUnicycleController();

    LTVUnicycleController(std::array<float, 3> Q,
                          std::array<float, 2> simple_Q,
                          std::array<float, 2> R,
                          Time input_delay = 0_msec,
                          LinearVelocity minimum_velocity = 0_mps);

    LTVUnicycleController(std::array<float, 3> Q,
                          std::array<float, 2> R,
                          Time input_delay = 0_msec,
                          LinearVelocity minimum_velocity = 0_mps);
};

} // namespace state_space
} // namespace lyfast
} // namespace blazing
