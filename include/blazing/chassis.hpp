#pragma once

#include <functional>
namespace blazing {

// simple struct to simplify passing around these values
template<typename DrivetrainType, typename TrackerType, typename TolerancesType>
struct Chassis {
    using drivetrainType = DrivetrainType;
    using trackerType = TrackerType;
    using tolerancesType = TolerancesType;

	DrivetrainType* drivetrain;
    TrackerType* tracker;

    TolerancesType tolerances;
};
} // namespace blazing
