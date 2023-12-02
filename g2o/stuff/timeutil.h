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

#ifndef G2O_TIMEUTIL_H
#define G2O_TIMEUTIL_H

#include <chrono>
#include <string>

#include "g2o_stuff_api.h"

/** @addtogroup utils **/
// @{

/** \file timeutil.h
 * \brief utility functions for handling time related stuff
 */

/// Executes code, only if secs are gone since last exec.
/// extended version, in which the current time is given, e.g., timestamp of IPC
/// message
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code)        \
  if (1) {                                          \
    static double s_lastDone_ = (currentTime);      \
    double s_now_ = (currentTime);                  \
    if (s_lastDone_ > s_now_) s_lastDone_ = s_now_; \
    if (s_now_ - s_lastDone_ > (secs)) {            \
      code;                                         \
      s_lastDone_ = s_now_;                         \
    }                                               \
  } else                                            \
    (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) DO_EVERY_TS(secs, g2o::get_time(), code)
#endif

#ifndef MEASURE_TIME
#define MEASURE_TIME(text, code)                                       \
  if (1) {                                                             \
    double _start_time_ = g2o::get_time();                             \
    code;                                                              \
    G2O_DEBUG("{} took {} sec", text, g2o::get_time() - _start_time_); \
  } else                                                               \
    (void)0
#endif

namespace g2o {

using seconds = std::chrono::duration<double>;

/**
 * return the current time in seconds since 1. Jan 1970
 */
inline double get_time() {
  return seconds{std::chrono::system_clock::now().time_since_epoch()}.count();
}

/**
 * return a monotonic increasing time which basically does not need to
 * have a reference point. Consider this for measuring how long some
 * code fragments required to execute.
 */
G2O_STUFF_API double get_monotonic_time();

/**
 * \brief Class to measure the time spent in a scope
 *
 * To use this class, e.g. to measure the time spent in a function,
 * just create and instance at the beginning of the function.
 */
class G2O_STUFF_API ScopeTime {
 public:
  ScopeTime(const char* title);
  ~ScopeTime();

 private:
  std::string _title;
  double _startTime;
};

}  // namespace g2o

#ifndef MEASURE_FUNCTION_TIME
#define MEASURE_FUNCTION_TIME g2o::ScopeTime scopeTime(__FUNCTION__)
#endif

// @}
#endif
