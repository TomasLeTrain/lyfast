#include "main.h"
#include "lyfast/geometry/cubicBezier.h"
#include "units/Vector2D.hpp"
#include <iostream>

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    std::cout << "hello world!" << std::endl;

    lyfast::geometry::CubicBezier cubic({ 10_Fin, 10_Fin },
                                        { 20_Fin, 20_Fin },
                                        { 30_Fin, 30_Fin },
                                        { 0_Fin, 0_Fin });
}
