#pragma once

#include "blazing/motions/distanceAtHeading.hpp"
#include "blazing/motions/moveTo.hpp"
#include "blazing/motions/turnTo.hpp"
#include "blazing/utils.hpp"
#include "motions/boomerang.hpp"
#include "units/Vector2D.hpp"
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

    using MoveToModifier = std::function<moveToType(moveToType&&)>;
    using TurnToModifier = std::function<turnToType(turnToType&&)>;
    using ArcModifier = std::function<arcType(arcType&&)>;
    using DistanceAtHeadingModifier =
      std::function<distanceAtHeadingType(distanceAtHeadingType&&)>;
    using BoomerangModifier = std::function<boomerangType(boomerangType&&)>;

    MoveToModifier m_moveToModifier = [](moveToType&& moveTo) {
        return std::move(moveTo);
    };
    TurnToModifier m_turnToModifier = [](turnToType&& turnTo) {
        return std::move(turnTo);
    };

    ArcModifier m_arcModifier = [](arcType&& arc) {
        return std::move(arc);
    };

    DistanceAtHeadingModifier m_distanceAtHeadingModifier =
      [](distanceAtHeadingType&& distanceAtHeading) {
          return std::move(distanceAtHeading);
      };
    BoomerangModifier m_boomerangModifier = [](boomerangType&& boomerang) {
        return std::move(boomerang);
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
        m_moveToModifier = customModifier;
    }

    void setTurnToModifier(TurnToModifier customModifier) {
        m_turnToModifier = customModifier;
    }

    void
    setDistanceAtHeadingModifier(DistanceAtHeadingModifier customModifier) {
        m_distanceAtHeadingModifier = customModifier;
    }

    void setBoomerangModifier(BoomerangModifier customModifier) {
        m_boomerangModifier = customModifier;
    }

    void setArcModifier(ArcModifier customModifier) {
        m_arcModifier = customModifier;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveToType moveTo(std::variant<Length, double, int> x,
                      std::variant<Length, double, int> y) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);

        return m_moveToModifier(
          std::move(blazing::moveTo(controllers, chassis, new_x, new_y)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveToType moveTo(units::V2Position point) {
        return m_moveToModifier(
          std::move(blazing::moveTo(controllers, chassis, point)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveToType moveTo(std::function<units::V2Position()> point_func) {
        return m_moveToModifier(
          std::move(blazing::moveTo(controllers, chassis, point_func)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(units::V2Position point) {
        return m_turnToModifier(
          std::move(blazing::turnTo(controllers, chassis, point)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(std::variant<Length, double, int> x,
                      std::variant<Length, double, int> y) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);

        return m_turnToModifier(
          std::move(blazing::turnTo(controllers, chassis, new_x, new_y)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(std::variant<Angle, double, int> heading) {
        Angle new_heading = castToUnit(heading, deg);

        return m_turnToModifier(
          std::move(blazing::turnTo(controllers, chassis, new_heading)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType
    distanceAtHeading(std::variant<Length, double, int> distance) {
        Length new_distance = castToUnit(distance, in);

        return m_distanceAtHeadingModifier(std::move(
          blazing::distanceAtHeading(controllers, chassis, new_distance)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType
    distanceAtHeading(std::variant<Length, double, int> distance,
                      std::variant<Angle, double, int> heading) {
        Length new_distance = castToUnit(distance, in);
        Angle new_heading = castToUnit(heading, deg);

        return m_distanceAtHeadingModifier(
          std::move(blazing::distanceAtHeading(controllers,
                                               chassis,
                                               new_distance,
                                               new_heading)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    arcType arc(std::variant<Angle, double, int> heading, auto radius) {
        Angle new_heading = castToUnit(heading, deg);
        return m_arcModifier(
          std::move(blazing::Arc(controllers, chassis, new_heading, radius)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    arcType arc(std::variant<Length, double, int> x,
                std::variant<Length, double, int> y,
                auto radius) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);
        return m_arcModifier(
          std::move(blazing::Arc(controllers, chassis, new_x, new_y, radius)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    arcType arc(units::V2Position target_point, auto radius) {
        return m_arcModifier(
          std::move(blazing::Arc(controllers, chassis, target_point, radius)));
    }

    // boomerang
    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(units::Pose pose) {
        return m_boomerangModifier(
          std::move(blazing::boomerang(controllers, chassis, pose)));
    }

    // boomerang
    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(std::function<units::Pose()> pose_func) {
        return m_boomerangModifier(
          std::move(blazing::boomerang(controllers, chassis, pose_func)));
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(std::variant<Length, double, int> x,
                            std::variant<Length, double, int> y,
                            std::variant<Angle, double, int> heading) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);
        Angle new_heading = castToUnit(heading, deg);

        return m_boomerangModifier(std::move(
          blazing::boomerang(controllers, chassis, new_x, new_y, new_heading)));
    }
};
} // namespace blazing
