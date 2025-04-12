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

/** @addtogroup utils **/
// @{

/** \file misc.h
 * \brief some general case utility functions
 *
 *  This file specifies some general case utility functions
 **/

namespace g2o {

/**
 * converts a number constant to a double constant at compile time
 * to avoid having to cast everything to avoid warnings.
 **/
inline constexpr double cst(long double v) { return (double)v; }

constexpr double const_pi() { return cst(3.14159265358979323846); }

/**
 * convert from degree to radian
 */
inline double deg2rad(double degree) {
  return degree * cst(0.01745329251994329576);
}

/**
 * convert from radian to degree
 */
inline double rad2deg(double rad) { return rad * cst(57.29577951308232087721); }

/**
 * normalize the angle
 */
inline double normalize_theta(double theta) {
  const double result = fmod(theta + const_pi(), 2.0 * const_pi());
  if (result <= 0.0) return result + const_pi();
  return result - const_pi();
}

/**
 * The following two functions are used to force linkage with static libraries.
 */
extern "C" {
typedef void (*ForceLinkFunction)(void);
}

struct ForceLinker {
  ForceLinker(ForceLinkFunction function) { (function)(); }
};

}  // namespace g2o

// @}

#endif
