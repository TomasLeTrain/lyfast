#pragma once

#include "blazing/motions/distanceAtHeading.hpp"
#include "blazing/motions/moveTo.hpp"
#include "blazing/motions/turnTo.hpp"
#include "blazing/utils.hpp"
#include "motions/boomerang.hpp"
#include <iterator>
#include <variant>

namespace blazing {

template<typename Chassis, typename Controllers>
class MotionBuilder {
  private:
    Chassis chassis;
    Controllers controllers;

    using moveToType = blazing::moveTo<Controllers,
                                       typename Chassis::drivetrainType,
                                       typename Chassis::trackerType,
                                       typename Chassis::tolerancesType>;
    using turnToType = blazing::turnTo<Controllers,
                                       typename Chassis::drivetrainType,
                                       typename Chassis::trackerType,
                                       typename Chassis::tolerancesType>;
    using arcType = blazing::Arc<Controllers,
                                 typename Chassis::drivetrainType,
                                 typename Chassis::trackerType,
                                 typename Chassis::tolerancesType>;
    using distanceAtHeadingType =
      blazing::distanceAtHeading<Controllers,
                                 typename Chassis::drivetrainType,
                                 typename Chassis::trackerType,
                                 typename Chassis::tolerancesType>;
    using boomerangType = blazing::boomerang<Controllers,
                                             typename Chassis::drivetrainType,
                                             typename Chassis::trackerType,
                                             typename Chassis::tolerancesType>;

    using MoveToModifier = std::function<moveToType(moveToType)>;
    using TurnToModifier = std::function<turnToType(turnToType)>;
    using ArcModifier = std::function<arcType(arcType)>;
    using DistanceAtHeadingModifier =
      std::function<distanceAtHeadingType(distanceAtHeadingType)>;
    using BoomerangModifier = std::function<boomerangType(boomerangType)>;

    MoveToModifier moveToModifier = [](moveToType moveTo) {
        return moveTo;
    };
    TurnToModifier turnToModifier = [](turnToType turnTo) {
        return turnTo;
    };

    ArcModifier arcModifier = [](arcType arc) {
        return arc;
    };

    DistanceAtHeadingModifier distanceAtHeadingModifier =
      [](distanceAtHeadingType distanceAtHeading) {
          return distanceAtHeading;
      };
    BoomerangModifier boomerangModifier = [](boomerangType boomerang) {
        return boomerang;
    };

    template<typename T>
    auto castToUnit(std::variant<T, double, int> variant, T units) {
        if (std::holds_alternative<T>(variant))
            return std::get<T>(variant);
        else if (std::holds_alternative<double>(variant))
            return std::get<double>(variant) * units;
        else
            return std::get<int>(variant) * units;
    }

  public:
    MotionBuilder(Chassis chassis, Controllers controllers)
        : chassis(chassis),
          controllers(controllers) {}

    void setMoveToModifier(MoveToModifier customModifier) {
        moveToModifier = customModifier;
    }

    void setTurnToModifier(TurnToModifier customModifier) {
        turnToModifier = customModifier;
    }

    void
    setDistanceAtHeadingModifier(DistanceAtHeadingModifier customModifier) {
        distanceAtHeadingModifier = customModifier;
    }

    void setBoomerangModifier(BoomerangModifier customModifier) {
        boomerangModifier = customModifier;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveToType moveTo(std::variant<Length, double, int> x,
                      std::variant<Length, double, int> y) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);

        return moveToModifier(
          blazing::moveTo(controllers, chassis, new_x, new_y));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(std::variant<Length, double, int> x,
                      std::variant<Length, double, int> y) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);

        return turnToModifier(
          blazing::turnTo(controllers, chassis, new_x, new_y));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(std::variant<Angle, double, int> heading) {
        Angle new_heading = castToUnit(heading, deg);

        return turnToModifier(
          blazing::turnTo(controllers, chassis, new_heading));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType
    distanceAtHeading(std::variant<Length, double, int> distance) {
        Length new_distance = castToUnit(distance, in);

        return distanceAtHeadingModifier(
          blazing::distanceAtHeading(controllers, chassis, new_distance));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType
    distanceAtHeading(std::variant<Length, double, int> distance,
                      std::variant<Angle, double, int> heading) {
        Length new_distance = castToUnit(distance, in);
        Angle new_heading = castToUnit(heading, deg);

        return distanceAtHeadingModifier(
          blazing::distanceAtHeading(controllers,
                                     chassis,
                                     new_distance,
                                     new_heading));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    arcType arc(std::variant<Angle, double, int> heading, double radius = 1.0) {
        Angle new_heading = castToUnit(heading, deg);
        return arcModifier(
          blazing::Arc(controllers, chassis, new_heading, radius));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    arcType arc(std::variant<Length, double, int> x,
                std::variant<Length, double, int> y,
                double radius = 1.0) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);
        return arcModifier(
          blazing::Arc(controllers, chassis, new_x, new_y, radius));
    }

    // boomerang
    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(units::Pose pose) {
        return boomerangModifier(
          blazing::boomerang(controllers, chassis, pose));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(std::variant<Length, double, int> x,
                            std::variant<Length, double, int> y,
                            std::variant<Angle, double, int> heading) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);
        Angle new_heading = castToUnit(heading, deg);

        return boomerangModifier(
          blazing::boomerang(controllers, chassis, new_x, new_y, new_heading));
    }
};
} // namespace blazing
