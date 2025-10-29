#include "main.h"
#include "lyfast/geometry/cubicBezier.hpp"
#include "lyfast/motion_profiling/constraints.hpp"
#include "lyfast/motion_profiling/mp.hpp"
#include "units/Vector2D.hpp"
#include <iostream>

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    std::cout << "hello world!" << std::endl;

    lyfast::geometry::CubicBezier cubic({ 2_in, 2_in },
                                        { 15_in, 5_in },
                                        { 5_in, 15_in },
                                        { 20_in, 14_in });

    lyfast::mp::RobotConstraints robot_constraints(10.5_in, 1.0);
    lyfast::mp::LinearConstraints linear_constraints(60_inps, 30_inps2, 30_inps2);
    lyfast::mp::AngularConstraints angular_constraints(1_rps,
                                                       1_rps2,
                                                       1_rps2);

    lyfast::mp::Constraints constraints(robot_constraints,
                                        linear_constraints,
                                        angular_constraints);

    lyfast::mp::Trajectory cubic_trajectory(&cubic,
                                            constraints,
                                            0_mps,
                                            0_mps,
                                            0.1_in);

    // print out final trajectory and debug info

    auto print =
      []<typename T>(std::string name, std::vector<T>& list, T target_units) {
          std::cout << name << "=\\left[";
          for (size_t i = 0; i < list.size(); i++) {
              if (i != 0) std::cout << ",";
              std::cout << list[i].convert(target_units);
          }
          std::cout << "\\right]" << std::endl;
      };

    print("a_{kin}", cubic_trajectory.max_kin_accel_debug, Finps2);
    print("a_{turn}", cubic_trajectory.max_turn_accel_debug, Finps2);
    print("d_{kin}", cubic_trajectory.max_kin_decel_debug, Finps2);
    print("d_{turn}", cubic_trajectory.max_turn_decel_debug, Finps2);

    print("v_{kin}", cubic_trajectory.max_kin_vel_debug, Finps);
    print("v_{turn}", cubic_trajectory.max_turn_vel_debug, Finps);
    print("v_{friction}", cubic_trajectory.max_friction_vel_debug, Finps);

    print("v_{forward}", cubic_trajectory.forwards_pass_debug, Finps);
    print("v_{backward}", cubic_trajectory.backwards_pass_debug, Finps);

    print("v_{final}", cubic_trajectory.final_vels_debug, Finps);

}
