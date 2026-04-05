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

    using MoveToModifier = std::function<void(moveToType*)>;
    using TurnToModifier = std::function<void(turnToType*)>;
    using ArcModifier = std::function<void(arcType*)>;
    using DistanceAtHeadingModifier =
      std::function<void(distanceAtHeadingType*)>;
    using BoomerangModifier = std::function<void(boomerangType*)>;

    MoveToModifier m_moveToModifier = [](moveToType* moveTo) -> void {};
    TurnToModifier m_turnToModifier = [](turnToType* turnTo) -> void {};
    ArcModifier m_arcModifier = [](arcType* arc) -> void {};
    DistanceAtHeadingModifier m_distanceAtHeadingModifier =
      [](distanceAtHeadingType* distanceAtHeading) -> void {};
    BoomerangModifier m_boomerangModifier =
      [](boomerangType* boomerang) -> void {};

    template<typename T>
    T castToUnit(std::variant<T, double, int> variant, T units) {
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

        auto motion = blazing::moveTo(controllers, chassis, new_x, new_y);

        m_moveToModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveToType moveTo(units::V2Position point) {
        auto motion = blazing::moveTo(controllers, chassis, point);

        m_moveToModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    moveToType moveTo(std::function<units::V2Position()> point_func) {
        auto motion = blazing::moveTo(controllers, chassis, point_func);

        m_moveToModifier(&motion);

        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(units::V2Position point) {
        auto motion = blazing::turnTo(controllers, chassis, point);
        m_turnToModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(std::variant<Length, double, int> x,
                      std::variant<Length, double, int> y) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);

        auto motion = blazing::turnTo(controllers, chassis, new_x, new_y);
        m_turnToModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    turnToType turnTo(std::variant<Angle, double, int> heading) {
        Angle new_heading = castToUnit(heading, deg);

        auto motion = blazing::turnTo(controllers, chassis, new_heading);
        m_turnToModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType
    distanceAtHeading(std::variant<Length, double, int> distance) {
        Length new_distance = castToUnit(distance, in);

        auto motion =
          blazing::distanceAtHeading(controllers, chassis, new_distance);
        m_distanceAtHeadingModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    distanceAtHeadingType
    distanceAtHeading(std::variant<Length, double, int> distance,
                      std::variant<Angle, double, int> heading) {
        Length new_distance = castToUnit(distance, in);
        Angle new_heading = castToUnit(heading, deg);

        auto motion = blazing::distanceAtHeading(controllers,
                                                 chassis,
                                                 new_distance,
                                                 new_heading);
        m_distanceAtHeadingModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    arcType arc(std::variant<Angle, double, int> heading, auto radius) {
        Angle new_heading = castToUnit(heading, deg);

        auto motion = blazing::Arc(controllers, chassis, new_heading, radius);
        m_arcModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    arcType arc(std::variant<Length, double, int> x,
                std::variant<Length, double, int> y,
                auto radius) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);

        auto motion = blazing::Arc(controllers, chassis, new_x, new_y, radius);
        m_arcModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    arcType arc(units::V2Position target_point, auto radius) {

        auto motion = blazing::Arc(controllers, chassis, target_point, radius);
        m_arcModifier(&motion);
        return motion;
    }

    // boomerang
    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(units::Pose pose) {
        auto motion = blazing::boomerang(controllers, chassis, pose);
        m_boomerangModifier(&motion);
        return motion;
    }

    // boomerang
    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(std::function<units::Pose()> pose_func) {
        auto motion = blazing::boomerang(controllers, chassis, pose_func);
        m_boomerangModifier(&motion);
        return motion;
    }

    [[nodiscard("motion won't be executed unless an executor is used!")]]
    boomerangType boomerang(std::variant<Length, double, int> x,
                            std::variant<Length, double, int> y,
                            std::variant<Angle, double, int> heading) {
        Length new_x = castToUnit(x, in);
        Length new_y = castToUnit(y, in);
        Angle new_heading = castToUnit(heading, deg);

        auto motion =
          blazing::boomerang(controllers, chassis, new_x, new_y, new_heading);
        m_boomerangModifier(&motion);
        return motion;
    }
};

} // namespace blazing
