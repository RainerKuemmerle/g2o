// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_TIMEUTIL_H
#define G2O_TIMEUTIL_H

#ifdef _WINDOWS
#include <time.h>
#else
#include <sys/time.h>
#endif

#include <string>

/** @addtogroup utils **/
// @{

/** \file timeutil.h
 * \brief utility functions for handling time related stuff
 */

/// Executes code, only if secs are gone since last exec.
/// extended version, in which the current time is given, e.g., timestamp of IPC message
#ifndef DO_EVERY_TS
#define DO_EVERY_TS(secs, currentTime, code) \
if (1) {\
  static double s_lastDone_ = (currentTime); \
  double s_now_ = (currentTime); \
  if (s_lastDone_ > s_now_) \
    s_lastDone_ = s_now_; \
  if (s_now_ - s_lastDone_ > (secs)) { \
    code; \
    s_lastDone_ = s_now_; \
  }\
} else \
  (void)0
#endif

/// Executes code, only if secs are gone since last exec.
#ifndef DO_EVERY
#define DO_EVERY(secs, code) DO_EVERY_TS(secs, g2o::get_time(), code)
#endif

#ifndef MEASURE_TIME
#define MEASURE_TIME(text, code) \
  if(1) { \
    double _start_time_ = g2o::get_time(); \
    code; \
    fprintf(stderr, "%s took %f sec\n", text, g2o::get_time() - _start_time_); \
  } else \
    (void) 0
#endif

namespace g2o {

#ifdef _WINDOWS
typedef struct timeval {
  long tv_sec;
  long tv_usec;
} timeval;
int gettimeofday(struct timeval *tv, struct timezone *tz);
#endif

/**
 * return the current time in seconds since 1. Jan 1970
 */
inline double get_time() 
{
  struct timeval ts;
  gettimeofday(&ts,0);
  return ts.tv_sec + ts.tv_usec*1e-6;
}

/**
 * \brief Class to measure the time spent in a scope
 *
 * To use this class, e.g. to measure the time spent in a function,
 * just create and instance at the beginning of the function.
 */
class ScopeTime {
  public: 
    ScopeTime(const char* title);
    ~ScopeTime();
  private:
    std::string _title;
    double _startTime;
};

} // end namespace

#define MEASURE_FUNCTION_TIME \
  g2o::ScopeTime scopeTime(__PRETTY_FUNCTION__)


// @}
#endif
