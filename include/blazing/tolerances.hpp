#pragma once

#include "pros/rtos.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include "utils.hpp"
#include <optional>

namespace blazing {

class ToleranceBase {
  protected:
    std::optional<bool> in_tolerance = std::nullopt;

    void update_in_tolerance(bool tolerance) {
        if (in_tolerance.has_value()) {
            in_tolerance = in_tolerance.value() && tolerance;
        } else {
            in_tolerance = tolerance;
        }
    }
};

template<typename T>
class ErrorTolerance : virtual ToleranceBase {
  private:
    std::optional<T> error_tolerance = std::nullopt;

  public:
    ErrorTolerance(T error_tolerance)
        : error_tolerance(error_tolerance) {}

    void setErrorTolerance(T error_tolerance) {
        this->error_tolerance = error_tolerance;
    }

    void errorToleranceUpdate(T error) {
        bool curr_tolerance_active = error_tolerance
                                       .transform([error](T tolerance) -> bool {
                                           return units::abs(error) < tolerance;
                                       })
                                       .value_or(false);
        update_in_tolerance(curr_tolerance_active);
    }
};

template<typename T>
class VelocityTolerance : virtual ToleranceBase {
  private:
    std::optional<Divided<T, Time>> velocity_tolerance = std::nullopt;

  public:
    VelocityTolerance(Divided<T, Time> velocity_tolerance)
        : velocity_tolerance(velocity_tolerance) {}

    void setVelocityTolerance(Divided<T, Time> velocity_tolerance) {
        this->velocity_tolerance = velocity_tolerance;
    }

    void velocityToleranceUpdate(Divided<T, Time> velocity) {
        bool curr_tolerance_active =
          velocity_tolerance
            .transform([velocity](Divided<T, Time> tolerance) -> bool {
                return units::abs(velocity) < tolerance;
            })
            .value_or(false);

        update_in_tolerance(curr_tolerance_active);
    }
};

// deduction guide to allow passing in velocity directly to resolve to vel *
// time
template<typename T>
VelocityTolerance(T) -> VelocityTolerance<Multiplied<T, Time>>;

class HalfCircleTolerance : virtual ToleranceBase {
  private:
    std::optional<Length> radius_tolerance = std::nullopt;
    std::optional<bool> prev_side = std::nullopt;

  public:
    HalfCircleTolerance(Length radius_tolerance)
        : radius_tolerance(radius_tolerance) {}

    void setHalfcircleTolerance(Length radius_tolerance) {
        this->radius_tolerance = radius_tolerance;
    }

    void halfcircleToleranceUpdate(units::V2Position pose,
                                   units::V2Position target,
                                   Angle theta) {
        bool side =
          radius_tolerance
            .transform([pose, target, theta](Length tolerance) -> bool {
                return (pose.y - target.y) * -units::sin(theta) >=
                       units::cos(theta) * (pose.x - target.y) + tolerance;
            })
            .value_or(false);

        if (prev_side == std::nullopt) prev_side = side;
        bool curr_tolerance_active = side != *prev_side;
        prev_side = side;

        update_in_tolerance(curr_tolerance_active);
    }
};

// allows inheriting all the functions from the tolerances
template<typename... ToleranceTypes>
    requires(std::is_base_of_v<ToleranceBase, ToleranceTypes> && ...)
class Tolerances : virtual ToleranceBase,
                   public ToleranceTypes... {
  private:
    std::optional<Time> tolerance_timestamp = std::nullopt;
    std::optional<Time> duration = std::nullopt;

  public:
    template<typename... U>
        requires(sizeof...(U) == sizeof...(ToleranceTypes) &&
                 (std::is_constructible_v<ToleranceTypes, U> && ...))
    Tolerances(Time duration, U&&... bases)
        : duration(duration),
          ToleranceTypes(std::forward<U>(bases))... {}

    void setDuration(Time duration) {
        this->duration = duration;
    }

    bool withinTolerance() {
        return in_tolerance.value_or(false);
    }

    // assumes each tolerance check has been performed
    bool finished() {
        if (withinTolerance()) {
            in_tolerance = std::nullopt;

            // set timestamp if it doesn't have one
            tolerance_timestamp = tolerance_timestamp.value_or(now());

            // enough time has passed
            if (duration
                  .transform([timestamp =
                                *this->tolerance_timestamp](Time time) -> bool {
                      return now() - timestamp > time;
                  })
                  .value_or(false)) {
                return true;
            }
            // not enough time has passed
            // return false but don't reset anything
            return false;
        }

        in_tolerance = std::nullopt;
        tolerance_timestamp = std::nullopt;

        return false;
    }
};

// deduction guide allows specifying tolerance types from constructor
template<typename... ToleranceTypes>
Tolerances(Time, ToleranceTypes&&...)
  -> Tolerances<std::remove_cvref_t<ToleranceTypes>...>;

// tolerance concepts
template<typename TolerancesType>
concept hasLinearTolerance =
  requires(TolerancesType tolerances) { tolerances.linear; };
template<typename TolerancesType>
concept hasLargeLinearTolerance =
  requires(TolerancesType tolerances) { tolerances.large_linear; };

template<typename TolerancesType>
concept hasAngularTolerance =
  requires(TolerancesType tolerances) { tolerances.angular; };
template<typename TolerancesType>
concept hasLargeAngularTolerance =
  requires(TolerancesType tolerances) { tolerances.large_angular; };

template<typename TolerancesType>
concept hasChainLinearTolerance =
  requires(TolerancesType tolerances) { tolerances.chain_linear; };
template<typename TolerancesType>
concept hasChainAngularTolerance =
  requires(TolerancesType tolerances) { tolerances.chain_angular; };

// linear errors
template<typename TolerancesType>
concept hasLinearErrorTolerance =
  requires(TolerancesType tolerances, Length error) {
      tolerances.linear.setErrorTolerance(error);
      tolerances.linear.errorToleranceUpdate(error);
  };

template<typename TolerancesType>
concept hasLargeLinearErrorTolerance =
  requires(TolerancesType tolerances, Length error) {
      tolerances.large_linear.setErrorTolerance(error);
      tolerances.large_linear.errorToleranceUpdate(error);
  };

template<typename TolerancesType>
concept hasChainLinearErrorTolerance =
  requires(TolerancesType tolerances, Length error) {
      tolerances.chain_linear.setErrorTolerance(error);
      tolerances.chain_linear.errorToleranceUpdate(error);
  };

// angular errors
template<typename TolerancesType>
concept hasAngularErrorTolerance =
  requires(TolerancesType tolerances, Angle error) {
      tolerances.angular.setErrorTolerance(error);
      tolerances.angular.errorToleranceUpdate(error);
  };

template<typename TolerancesType>
concept hasLargeAngularErrorTolerance =
  requires(TolerancesType tolerances, Angle error) {
      tolerances.large_angular.setErrorTolerance(error);
      tolerances.large_angular.errorToleranceUpdate(error);
  };

template<typename TolerancesType>
concept hasChainAngularErrorTolerance =
  requires(TolerancesType tolerances, Angle error) {
      tolerances.chain_angular.setErrorTolerance(error);
      tolerances.chain_angular.errorToleranceUpdate(error);
  };

// linear velocity
template<typename TolerancesType>
concept hasLinearVelocityTolerance =
  requires(TolerancesType tolerances, LinearVelocity velocity) {
      tolerances.linear.setVelocityTolerance(velocity);
      tolerances.linear.velocityToleranceUpdate(velocity);
  };
template<typename TolerancesType>
concept hasLargeLinearVelocityTolerance =
  requires(TolerancesType tolerances, LinearVelocity velocity) {
      tolerances.large_linear.setVelocityTolerance(velocity);
      tolerances.large_linear.velocityToleranceUpdate(velocity);
  };
template<typename TolerancesType>
concept hasChainLinearVelocityTolerance =
  requires(TolerancesType tolerances, LinearVelocity velocity) {
      tolerances.chain_linear.setVelocityTolerance(velocity);
      tolerances.chain_linear.velocityToleranceUpdate(velocity);
  };

// angular velocity
template<typename TolerancesType>
concept hasAngularVelocityTolerance =
  requires(TolerancesType tolerances, AngularVelocity velocity) {
      tolerances.angular.setVelocityTolerance(velocity);
      tolerances.angular.velocityToleranceUpdate(velocity);
  };

template<typename TolerancesType>
concept hasLargeAngularVelocityTolerance =
  requires(TolerancesType tolerances, AngularVelocity velocity) {
      tolerances.large_angular.setVelocityTolerance(velocity);
      tolerances.large_angular.velocityToleranceUpdate(velocity);
  };

template<typename TolerancesType>
concept hasChainAngularVelocityTolerance =
  requires(TolerancesType tolerances, AngularVelocity velocity) {
      tolerances.chain_angular.setVelocityTolerance(velocity);
      tolerances.chain_angular.velocityToleranceUpdate(velocity);
  };

// half circle
template<typename TolerancesType>
concept hasLinearHalfcircleTolerance = requires(TolerancesType tolerances,
                                                Length tolerance,
                                                units::V2Position pose,
                                                units::V2Position target,
                                                Angle theta) {
    tolerances.linear.setHalfcircleTolerance(tolerance);
    tolerances.linear.halfcircleToleranceUpdate(pose, target, theta);
};

template<typename TolerancesType>
concept hasLargeLinearHalfcircleTolerance = requires(TolerancesType tolerances,
                                                     Length tolerance,
                                                     units::V2Position pose,
                                                     units::V2Position target,
                                                     Angle theta) {
    tolerances.large_linear.setHalfcircleTolerance(tolerance);
    tolerances.large_linear.halfcircleToleranceUpdate(pose, target, theta);
};

template<typename TolerancesType>
concept hasChainLinearHalfcircleTolerance = requires(TolerancesType tolerances,
                                                     Length tolerance,
                                                     units::V2Position pose,
                                                     units::V2Position target,
                                                     Angle theta) {
    tolerances.chain_linear.setHalfcircleTolerance(tolerance);
    tolerances.chain_linear.halfcircleToleranceUpdate(pose, target, theta);
};

struct TolerancesGroup {
    // updates
    virtual void linearErrorToleranceUpdate(Length error) {}

    virtual void linearVelocityToleranceUpdate(LinearVelocity velocity) {}

    virtual void linearHalfcircleToleranceUpdate(units::V2Position pose,
                                                 units::V2Position target,
                                                 Angle theta) {}

    virtual void angularErrorToleranceUpdate(Angle error) {}

    virtual void angularVelocityToleranceUpdate(AngularVelocity velocity) {}

    virtual ~TolerancesGroup() = default;
};

template<typename LinearTolerances, typename AngularTolerances>
struct SimpleTolerances : public TolerancesGroup {
    LinearTolerances linear;
    AngularTolerances angular;

    SimpleTolerances(LinearTolerances linear, AngularTolerances angular)
        : linear(linear),
          angular(angular) {}

    void linearErrorToleranceUpdate(Length error) override {
        if constexpr (hasLinearErrorTolerance<SimpleTolerances>) {
            linear.errorToleranceUpdate(error);
        }
    }

    void linearVelocityToleranceUpdate(LinearVelocity velocity) override {
        if constexpr (hasLinearVelocityTolerance<SimpleTolerances>) {
            linear.velocityToleranceUpdate(velocity);
        }
    };

    void linearHalfcircleToleranceUpdate(units::V2Position pose,
                                         units::V2Position target,
                                         Angle theta) override {
        if constexpr (hasLinearHalfcircleTolerance<SimpleTolerances>) {
            linear.halfcircleToleranceUpdate(pose, target, theta);
        }
    }

    void angularErrorToleranceUpdate(Angle error) override {
        if constexpr (hasAngularErrorTolerance<SimpleTolerances>) {
            angular.errorToleranceUpdate(error);
        }
    }

    void angularVelocityToleranceUpdate(AngularVelocity velocity) override {
        if constexpr (hasAngularVelocityTolerance<SimpleTolerances>) {
            angular.velocityToleranceUpdate(velocity);
        }
    }
};

template<typename LinearTolerances,
         typename AngularTolerances,
         typename LargeLinearTolerances,
         typename LargeAngularTolerances>
struct normalLargeTolerances
    : public SimpleTolerances<LinearTolerances, AngularTolerances> {

    LargeLinearTolerances large_linear;
    LargeAngularTolerances large_angular;

    using inherited_type =
      SimpleTolerances<LinearTolerances, AngularTolerances>;

    normalLargeTolerances(LinearTolerances linear,
                          AngularTolerances angular,
                          LargeLinearTolerances large_linear,
                          LargeAngularTolerances large_angular)
        : inherited_type(linear, angular),
          large_linear(large_linear),
          large_angular(large_angular) {}

    void linearErrorToleranceUpdate(Length error) override {
        inherited_type::linearErrorToleranceUpdate(error);

        if constexpr (hasLargeLinearErrorTolerance<normalLargeTolerances>) {
            large_linear.errorToleranceUpdate(error);
        }
    }

    void linearVelocityToleranceUpdate(LinearVelocity velocity) override {
        inherited_type::linearVelocityToleranceUpdate(velocity);

        if constexpr (hasLargeLinearVelocityTolerance<normalLargeTolerances>) {
            large_linear.velocityToleranceUpdate(velocity);
        }
    };

    void linearHalfcircleToleranceUpdate(units::V2Position pose,
                                         units::V2Position target,
                                         Angle theta) override {
        inherited_type::linearHalfcircleToleranceUpdate(pose, target, theta);

        if constexpr (hasLargeLinearHalfcircleTolerance<
                        normalLargeTolerances>) {
            large_linear.halfcircleToleranceUpdate(pose, target, theta);
        }
    }

    void angularErrorToleranceUpdate(Angle error) override {
        inherited_type::angularErrorToleranceUpdate(error);

        if constexpr (hasLargeAngularErrorTolerance<normalLargeTolerances>) {
            large_angular.errorToleranceUpdate(error);
        }
    }

    void angularVelocityToleranceUpdate(AngularVelocity velocity) override {
        inherited_type::angularVelocityToleranceUpdate(velocity);

        if constexpr (hasLargeAngularVelocityTolerance<normalLargeTolerances>) {
            large_angular.velocityToleranceUpdate(velocity);
        }
    }
};

template<typename LinearTolerances,
         typename AngularTolerances,
         typename LargeLinearTolerances,
         typename LargeAngularTolerances,
         typename LinearChainTolerances,
         typename AngularChainTolerances>
struct normalLargeChainTolerances
    : public normalLargeTolerances<LinearTolerances,
                                   AngularTolerances,
                                   LargeLinearTolerances,
                                   LargeAngularTolerances> {
    LinearChainTolerances chain_linear;
    AngularChainTolerances chain_angular;

    using inherited_type = normalLargeTolerances<LinearTolerances,
                                                 AngularTolerances,
                                                 LargeLinearTolerances,
                                                 LargeAngularTolerances>;

    normalLargeChainTolerances(LinearTolerances linear,
                               AngularTolerances angular,
                               LargeLinearTolerances large_linear,
                               LargeAngularTolerances large_angular,
                               LinearChainTolerances chain_linear,
                               AngularChainTolerances chain_angular)
        : inherited_type(linear, angular, large_linear, large_angular),
          chain_linear(chain_linear),
          chain_angular(chain_angular) {}

    void linearErrorToleranceUpdate(Length error) override {
        inherited_type::linearErrorToleranceUpdate(error);

        if constexpr (hasChainLinearErrorTolerance<
                        normalLargeChainTolerances>) {
            chain_linear.errorToleranceUpdate(error);
        }
    }

    void linearVelocityToleranceUpdate(LinearVelocity velocity) override {
        inherited_type::linearVelocityToleranceUpdate(velocity);

        if constexpr (hasChainLinearVelocityTolerance<
                        normalLargeChainTolerances>) {
            chain_linear.velocityToleranceUpdate(velocity);
        }
    };

    void linearHalfcircleToleranceUpdate(units::V2Position pose,
                                         units::V2Position target,
                                         Angle theta) override {
        inherited_type::linearHalfcircleToleranceUpdate(pose, target, theta);

        if constexpr (hasChainLinearHalfcircleTolerance<
                        normalLargeChainTolerances>) {
            chain_linear.halfcircleToleranceUpdate(pose, target, theta);
        }
    }

    void angularErrorToleranceUpdate(Angle error) override {
        inherited_type::angularErrorToleranceUpdate(error);

        if constexpr (hasChainAngularErrorTolerance<
                        normalLargeChainTolerances>) {
            chain_angular.errorToleranceUpdate(error);
        }
    }

    void angularVelocityToleranceUpdate(AngularVelocity velocity) override {
        inherited_type::angularVelocityToleranceUpdate(velocity);

        if constexpr (hasChainAngularVelocityTolerance<
                        normalLargeChainTolerances>) {
            chain_angular.velocityToleranceUpdate(velocity);
        }
    }
};

} // namespace blazing
