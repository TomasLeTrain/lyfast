#pragma once

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/feedback/feedback.hpp"
#include "blazing/utils.hpp"
#include "units/Angle.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"

namespace blazing {
namespace lyfast {

struct PathPoseFeedbackT {
    units::Pose pose;
    DifferentialSpeeds velocities;
};

// generic path pose feedback controller (ramsete, stanley, lqr, etc.)
template<typename Controller>
concept hasPathPoseFeedback = Feedback<decltype(Controller::path_pose_feedback),
                                       PathPoseFeedbackT,
                                       DifferentialSpeeds>;

// wrapper for path pose feedback controllers
template<typename Controller>
    requires Feedback<Controller, PathPoseFeedbackT, DifferentialSpeeds>
struct PathPoseFeedbackController : virtual ControllerBase {
  public:
    Controller path_pose_feedback;

    PathPoseFeedbackController(Controller path_pose_feedback_controller)
        : path_pose_feedback(path_pose_feedback_controller) {}

    // creates a copy of the controller with different linear feedback
    // controller
    void
    set_PathPoseFeedbackController(Controller path_pose_feedback_controller) {
        this->path_pose_feedback = path_pose_feedback_controller;
    }
};

class RamsetteController {
  public:
    using zeta_unit = Divided<Number, Angle>;
    using beta_unit = Exponentiated<Divided<Angle, Length>, std::ratio<2>>;

  private:
    // zeta = 1 / rad
    zeta_unit zeta = 1 * (1 / rad);
    // beta = rad^2 / length^2
    beta_unit beta = 0.5 * units::pow<2>(rad / m);

  public:
    // TODO: what does changing this exactly do?
    void setZeta(zeta_unit new_zeta) {
        zeta = new_zeta;
    }

    // TODO: what does changing this exactly do?
    void setBeta(zeta_unit new_beta) {
        zeta = new_beta;
    }

    zeta_unit getZeta() {
        return zeta;
    }

    zeta_unit getBeta() {
        return zeta;
    }

    DifferentialSpeeds update(PathPoseFeedbackT state,
                              PathPoseFeedbackT reference,
                              Time duration) {
        // measurement is current pose and velocities
        // target is desired pose and velocities

        const units::V2Position local_error =
          (reference.pose - state.pose).rotatedBy(-state.pose.orientation);
        const Angle angle_error =
          angleError(reference.pose.orientation, state.pose.orientation);

        auto reference_vels = reference.velocities;

        // k = 1 / time
        const Frequency k =
          2.0 * zeta *
          units::sqrt(units::square(reference_vels.angular_velocity) +
                      beta * units::square(reference_vels.linear_velocity));

        DifferentialSpeeds new_speeds;

        // v_new = cos(e_theta) * v + k * e_x
        new_speeds.linear_velocity =
          units::cos(angle_error) * reference_vels.linear_velocity +
          k * local_error.x;

        // w_new = w + k * e_theta + beta * v * sinc(e_theta) * e_y
        new_speeds.angular_velocity = reference_vels.angular_velocity +
                                      k * angle_error +
                                      beta * reference_vels.linear_velocity *
                                        sinc(angle_error) * local_error.y;
        return new_speeds;
    }
};

class StanleyController {
  public:
    // larger values lead to more aggressive angle correction
    Divided<Number, Time> m_k = 1 / sec;

    DifferentialSpeeds update(PathPoseFeedbackT state,
                              PathPoseFeedbackT reference,
                              Time duration) {
        // local error with respect to the reference (reference is the origin)
        auto reference_local_error =
          (state.pose - reference.pose).rotatedBy(-reference.pose.orientation);

        Length crosstrack_error = reference_local_error.y;
        const Angle angle_error =
          angleError(reference.pose.orientation, state.pose.orientation);

        Angle target_steering_error =
          angle_error + units::atan(m_k * crosstrack_error /
                                    reference.velocities.linear_velocity);

        // TODO: need generic control law support to convert steering angle into
        // angular velocity feedback
        Divided<Number, Time> angular_k = 1 / sec;

        AngularVelocity new_angular_velocity =
          target_steering_error * angular_k +
          reference.velocities.angular_velocity;

        // TODO: no feedback on the linear velocity?
        LinearVelocity new_linear_velocity =
          reference.velocities.linear_velocity;

        DifferentialSpeeds new_speeds = { new_linear_velocity,
                                          new_angular_velocity };
        return new_speeds;
    }
};

} // namespace lyfast
} // namespace blazing
