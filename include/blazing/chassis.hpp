#pragma once

namespace blazing {

// simple struct to simplify passing around these values
template<typename DrivetrainType, typename TrackerType, typename TolerancesType>
struct Chassis {
    using drivetrainType = DrivetrainType;
    using trackerType = TrackerType;
    using tolerancesType = TolerancesType;

    // WARNING: it is assumed drivetrain and tracker are global variables or
    // have lifetimes thorughout the entire program duration
    DrivetrainType& drivetrain;
    TrackerType& tracker;

    TolerancesType tolerances;
};
} // namespace blazing
