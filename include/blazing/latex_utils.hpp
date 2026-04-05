#pragma once

#include "units/Vector2D.hpp"
#include "units/units.hpp"
#include <functional>
#include <utility>
#include <vector>

namespace blazing {
// prints pair of floats as (x,y) coordinate pair
// NOTE: does not leave newline
void printPairAsLatex(std::pair<float, float> data);

// calls nu  pair of floats as (x,y) coordinate pair
void printPairListAsLatex(
  size_t size,
  std::function<std::pair<float, float>(size_t)> num_func);

// prints num_func as a pair size # of times, adds \left[ and
// \right] as well as commas in bewtween
void printListAsLatex(size_t size, std::function<float(size_t)> num_func);

// prints data list as latex list
// useful for integer data types
template<typename T>
void printListAsLatex(std::vector<T> data) {
    printListAsLatex(data.size(), [&](size_t i) -> float {
        return static_cast<float>(data[i]);
    });
}

// prints data list as latex list
// useful for integer data types
template<isQuantity T>
void printQuantityVectorAsLatex(std::vector<T> data, T unit = T { 1 }) {
    printListAsLatex(data.size(), [&](size_t i) -> float {
        return data[i].convert(unit);
    });
}

// prints vector as (x,y) coord pairs in latex format.
template<isQuantity P, isQuantity Q>
void printPairQuantityAsLatex(const std::pair<P, Q>& data,
                              P first_unit = P { 1 },
                              Q second_unit = Q { 1 }) {
    printPairAsLatex(
      { data.first.convert(first_unit), data.second.convert(second_unit) });
}

// prints vector as (x,y) coord pairs in a list, latex format. converted to the
// respective units
template<isQuantity P, isQuantity Q>
void printPairQuantitiesAsLatex(const std::vector<std::pair<P, Q>>& data,
                                P first_unit = P { 1 },
                                Q second_unit = Q { 1 }) {
    printPairListAsLatex(data.size(), [&](size_t i) -> std::pair<float, float> {
        return { data[i].first.convert(first_unit),
                 data[i].second.convert(second_unit) };
    });
}

// prints single vector as a latex coordinate pair
template<isQuantity T>
void printVectorAsLatex(const units::Vector2D<T>& vec, T unit = T { 1 }) {
    printPairAsLatex({ vec.x.convert(unit), vec.y.convert(unit) });
}

// prints vectors as latex coordinate pair in a list
template<isQuantity T>
void printVectorsAsLatex(const std::vector<units::Vector2D<T>>& data,
                         T unit = T { 1 }) {
    printPairListAsLatex(data.size(), [&](size_t i) -> std::pair<float, float> {
        return { data[i].x.convert(unit), data[i].y.convert(unit) };
    });
}

} // namespace blazing
