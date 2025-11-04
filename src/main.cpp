#include "main.h"
#include "lyfast/geometry/cubicBezier.hpp"
#include "lyfast/geometry/curve.hpp"
#include "lyfast/geometry/line.hpp"
#include "lyfast/geometry/spline.hpp"
#include "lyfast/motion_profiling/constraints.hpp"
#include "lyfast/motion_profiling/mp.hpp"
#include "pros/apix.h"
#include "units/Vector2D.hpp"
#include <iostream>

void initialize() {
    // pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void spline_test() {
    blazing::lyfast::geometry::CubicBezier first_cubic({ 15.35_in, 47.2_in },
                                                       { 23.2_in, 47.2_in },
                                                       { 23.2_in, 35_in },
                                                       { 15.1_in, 35_in });
    blazing::lyfast::geometry::Line line({ 15.1_in, 35_in }, { -40_in, 35_in });
    blazing::lyfast::geometry::CubicBezier second_cubic({ -40_in, 35_in },
                                                        { -50_in, 35_in },
                                                        { -44_in, 47.2_in },
                                                        { -56_in, 47.2_in });

    blazing::lyfast::geometry::Spline spline(
      { &first_cubic, &line, &second_cubic });
    // lyfast::geometry::Spline spline({ &first_cubic, &line, &second_cubic });

    blazing::lyfast::mp::RobotConstraints robot_constraints(10.5_in,
                                                            0.1,
                                                            3.25_in,
                                                            450_rpm,
                                                            13_lb,
                                                            6.0f);
    blazing::lyfast::mp::LinearConstraints linear_constraints(90_inps,
                                                              8.513_mps2,
                                                              1.25 * 8.513_mps2);
    blazing::lyfast::mp::AngularConstraints angular_constraints(
      (rad * 74_inps / 5.25_in),
      0.05_rps2,
      0.05_rps2);

    blazing::lyfast::mp::Constraints constraints(robot_constraints,
                                                 linear_constraints,
                                                 angular_constraints);

    blazing::lyfast::mp::Trajectory cubic_trajectory(&spline,
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
    //
    print("v_{kin}", cubic_trajectory.max_kin_vel_debug, Finps);
    print("v_{turn}", cubic_trajectory.max_turn_vel_debug, Finps);
    print("v_{friction}", cubic_trajectory.max_friction_vel_debug, Finps);

    print("v_{forward}", cubic_trajectory.forwards_pass_debug, Finps);
    print("v_{backward}", cubic_trajectory.backwards_pass_debug, Finps);

    print("v_{final}", cubic_trajectory.final_vels_debug, Finps);

    std::cout << "l_{times}=\\left[";
    for (auto& point : cubic_trajectory.points) {
        std::cout << point.travel_time.convert(sec) << ",";
    }
    std::cout << "\\right]" << std::endl;

    std::cout << "l_{points}=\\left[";
    for (auto& point : cubic_trajectory.points) {
        std::cout << "\\left(" << point.point.x.convert(in) << ","
                  << point.point.y.convert(in) << "\\right),";
    }
    std::cout << "\\right]" << std::endl;

    std::cout << "l_{headings}=\\left[";
    for (auto& point : cubic_trajectory.points) {
        std::cout << point.heading.internal() << ",";
    }
    std::cout << "\\right]" << std::endl;
}

// void cubic_test() {
//     std::cout << "hello world!" << std::endl;
//
//     blazing::lyfast::geometry::CubicBezier cubic({ 2_in, 2_in },
//                                                  { 5_in, 15_in },
//                                                  { 20_in, 14_in },
//                                                  { 15_in, 5_in });
//
//     blazing::lyfast::mp::RobotConstraints robot_constraints(10.5_in,
//                                                             0.1,
//                                                             3.25_in,
//                                                             450_rpm,
//                                                             13_lb,
//                                                             6.0f);
//     blazing::lyfast::mp::LinearConstraints linear_constraints(90_inps,
//                                                               60_inps2,
//                                                               60_inps2);
//     blazing::lyfast::mp::AngularConstraints angular_constraints(0.8_rps,
//                                                                 1_rps2,
//                                                                 1_rps2);
//
//     blazing::lyfast::mp::Constraints constraints(robot_constraints,
//                                                  linear_constraints,
//                                                  angular_constraints);
//
//     blazing::lyfast::mp::Trajectory cubic_trajectory(&cubic,
//                                                      constraints,
//                                                      0_mps,
//                                                      0_mps,
//                                                      0.1_in);
//
//     // print out final trajectory and debug info
//
//     auto print =
//       []<typename T>(std::string name, std::vector<T>& list, T target_units)
//       {
//           std::cout << name << "=\\left[";
//           for (size_t i = 0; i < list.size(); i++) {
//               if (i != 0) std::cout << ",";
//               std::cout << list[i].convert(target_units);
//           }
//           std::cout << "\\right]" << std::endl;
//       };
//
//     print("a_{kin}", cubic_trajectory.max_kin_accel_debug, Finps2);
//     print("a_{turn}", cubic_trajectory.max_turn_accel_debug, Finps2);
//     print("d_{kin}", cubic_trajectory.max_kin_decel_debug, Finps2);
//     print("d_{turn}", cubic_trajectory.max_turn_decel_debug, Finps2);
//
//     print("v_{kin}", cubic_trajectory.max_kin_vel_debug, Finps);
//     print("v_{turn}", cubic_trajectory.max_turn_vel_debug, Finps);
//     print("v_{friction}", cubic_trajectory.max_friction_vel_debug, Finps);
//
//     print("v_{forward}", cubic_trajectory.forwards_pass_debug, Finps);
//     print("v_{backward}", cubic_trajectory.backwards_pass_debug, Finps);
//
//     print("v_{final}", cubic_trajectory.final_vels_debug, Finps);
//
//     std::cout << "l_{times}=\\left[";
//     for (auto& point : cubic_trajectory.points) {
//         std::cout << point.travel_time.internal() << ",";
//     }
//     std::cout << "\\right]" << std::endl;
//
//     std::cout << "l_{points}=\\left[";
//     for (auto& point : cubic_trajectory.points) {
//         std::cout << "\\left(" << point.point.x.convert(in) << ","
//                   << point.point.y.convert(in) << "\\right),";
//     }
//     std::cout << "\\right]" << std::endl;
//
//     std::cout << "l_{headings}=\\left[";
//     for (auto& point : cubic_trajectory.points) {
//         std::cout << point.heading.internal() << ",";
//     }
//     std::cout << "\\right]" << std::endl;
// }

void opcontrol() {
    spline_test();
}
