// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_STUFF_MISC_H
#define G2O_STUFF_MISC_H

#include <cmath>
#include <memory>

#include "g2o/config.h"
#include "macros.h"

/** @addtogroup utils **/
// @{

/** \file misc.h
 * \brief some general case utility functions
 *
 *  This file specifies some general case utility functions
 **/

namespace g2o {

/** Helper class to sort pair based on first elem */
template <class T1, class T2, class Pred = std::less<T1> >
struct CmpPairFirst {
  bool operator()(const std::pair<T1, T2>& left,
                  const std::pair<T1, T2>& right) {
    return Pred()(left.first, right.first);
  }
};

/**
 * helper function for creating an object in a unique_ptr.
 */
template <typename T, typename... ArgTs>
std::unique_ptr<T> make_unique(ArgTs&&... args) {
  return std::unique_ptr<T>(new T(std::forward<ArgTs>(args)...));
};

/**
 * converts a number constant to a number_t constant at compile time
 * to avoid having to cast everything to avoid warnings.
 **/
inline constexpr number_t cst(long double value) {
  return static_cast<number_t>(value);
}

constexpr number_t const_pi() { return cst(3.14159265358979323846); }

/**
 * return the square value
 */
template <typename T>
inline T square(T value) {
  return value * value;
}

/**
 * return the hypot of x and y
 */
template <typename T>
inline T hypot(T x, T y) {  // NOLINT
  return static_cast<T>(std::sqrt(x * x + y * y));
}

/**
 * return the squared hypot of x and y
 */
template <typename T>
inline T hypot_sqr(T x, T y) {  // NOLINT
  return x * x + y * y;
}

/**
 * convert from degree to radian
 */
inline number_t deg2rad(number_t degree) {
  return degree * cst(0.01745329251994329576);
}

/**
 * convert from radian to degree
 */
inline number_t rad2deg(number_t rad) {
  return rad * cst(57.29577951308232087721);
}

/**
 * normalize the angle
 */
inline number_t normalize_theta(number_t theta) {
  const number_t result = fmod(theta + const_pi(), 2.0 * const_pi());
  if (result <= 0.0) return result + const_pi();
  return result - const_pi();
}

/**
 * inverse of an angle, i.e., +180 degree
 */
inline number_t inverse_theta(number_t theta) {
  return normalize_theta(theta + const_pi());
}

/**
 * average two angles
 */
inline number_t average_angle(number_t theta1, number_t theta2) {
  number_t x_coord = std::cos(theta1) + std::cos(theta2);
  number_t y_coord = std::sin(theta1) + std::sin(theta2);
  if (x_coord == 0 && y_coord == 0) return 0;
  return std::atan2(y_coord, x_coord);
}

/**
 * sign function.
 * @return the sign of value. +1 for value > 0, -1 for value < 0, 0 for value ==
 * 0
 */
template <typename T>
inline int sign(T value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

/**
 * clamp value to the interval [lower, upper]
 */
template <typename T>
inline T clamp(T lower, T value, T upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

/**
 * wrap value to be in the interval [lower, upper]
 */
template <typename T>
inline T wrap(T lower, T value, T upper) {
  T intervalWidth = upper - lower;
  while (value < lower) value += intervalWidth;
  while (value > upper) value -= intervalWidth;
  return value;
}

/**
 * tests whether there is a NaN in the array
 */
inline bool arrayHasNaN(const number_t* array, int size,
                        int* nanIndex = nullptr) {
  for (int i = 0; i < size; ++i)
    if (g2o_isnan(array[i])) {
      if (nanIndex) *nanIndex = i;
      return true;
    }
  return false;
}

/**
 * The following two functions are used to force linkage with static libraries.
 */
extern "C" {
using ForceLinkFunction = void (*)();
}

struct ForceLinker {
  explicit ForceLinker(ForceLinkFunction function) { (function)(); }
};

}  // namespace g2o

// @}

#endif
