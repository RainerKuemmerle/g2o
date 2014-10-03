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

#include "tictoc.h"

#include "timeutil.h"

#include <map>
#include <limits>
#include <algorithm>
#include <vector>
#include <cstdio>
#include <cstdlib>

namespace g2o {

  /**
   * \brief Internal structure of the tictoc profiling
   */
  struct TicTocElement
  {
    double ticTime;                         ///< the time of the last tic
    double totalTime;                       ///< the total time of this part of the algorithm
    int numCalls;                           ///< the number of calls
    double minTime;
    double maxTime;
    double exponentialMovingAverage;        ///< exponential moving average with alpha = 0.01
    std::string algorithmPart;              ///< name / description of the code block
    bool clockIsRunning;
    TicTocElement() :
      ticTime(0.), totalTime(0.), numCalls(0),
      minTime(std::numeric_limits<double>::max()),
      maxTime(0.), exponentialMovingAverage(0.),
      clockIsRunning(true)
    {}
    bool operator<(const TicTocElement& other) const
    {
      return totalTime < other.totalTime;
    }
  };
  typedef std::map<std::string, TicTocElement> TicTocMap;

  /**
   * \brief helper for printing the struct at the end of the lifetime of the program
   */
  struct TicTocInitializer
  {
    TicTocMap tictocElements;
    bool enabled;
    TicTocInitializer()
    {
      enabled = getenv("G2O_ENABLE_TICTOC") != NULL;
    }
    ~TicTocInitializer()
    {
      if (!enabled) {
        return;
      }

      if (tictocElements.size() > 0) {
        int longestName = 0;
        // sort the elements according to the total time and print a table
        std::vector<TicTocElement> sortedElements;
        sortedElements.reserve(tictocElements.size());
        for (TicTocMap::const_iterator it = tictocElements.begin(); it != tictocElements.end(); ++it) {
          if (it->second.numCalls == 0)
            continue;
          longestName = std::max(longestName, (int)it->first.size());
          sortedElements.push_back(it->second);
        }
        std::sort(sortedElements.begin(), sortedElements.end());

        longestName += 4;

        // now print the table to stdout
        printf("------------------------------------------\n");
        printf("|          TICTOC STATISTICS             |\n");
        printf("------------------------------------------\n");
        for(std::vector<TicTocElement>::const_iterator it = sortedElements.begin(); it != sortedElements.end(); ++it) {
          double avgTime = it->totalTime / it->numCalls;
          printf("%s", it->algorithmPart.c_str());
          for (int i = it->algorithmPart.size(); i < longestName; ++i)
            putchar(' ');
          printf("numCalls= %d\t total= %.4f\t avg= %.4f\t min= %.4f\t max= %.4f\t ema= %.4f\n",
              it->numCalls, it->totalTime, avgTime, it->minTime, it->maxTime, it->exponentialMovingAverage);
        }
        printf("------------------------------------------\n");
      }
    }
  };

  double tictoc(const char* algorithmPart)
  {
    static TicTocInitializer initializer;
    if (! initializer.enabled)
      return 0.;

    TicTocMap& tictocElements = initializer.tictocElements;
    static double alpha = 0.01;
    double now = get_monotonic_time();

    double dt = 0.;
    TicTocMap::iterator foundIt = tictocElements.find(algorithmPart);
    if (foundIt == tictocElements.end()) {
      // insert element
      TicTocElement e;
      e.ticTime = now;
      e.algorithmPart = algorithmPart;
      tictocElements[e.algorithmPart] = e;
    } else {
      if (foundIt->second.clockIsRunning) {
        dt = now - foundIt->second.ticTime;
        foundIt->second.totalTime += dt;
        foundIt->second.minTime = std::min(foundIt->second.minTime, dt);
        foundIt->second.maxTime = std::max(foundIt->second.maxTime, dt);
        if (foundIt->second.numCalls == 0)
          foundIt->second.exponentialMovingAverage = dt;
        else
          foundIt->second.exponentialMovingAverage = (1. - alpha) * foundIt->second.exponentialMovingAverage + alpha*dt;
        foundIt->second.numCalls++;
      } else {
        foundIt->second.ticTime = now;
      }
      foundIt->second.clockIsRunning = !foundIt->second.clockIsRunning;
    }
    return dt;
  }

  ScopedTictoc::ScopedTictoc(const char* algorithmPart) :
    _algorithmPart(algorithmPart)
  {
    tictoc(_algorithmPart.c_str());
  }

  ScopedTictoc::~ScopedTictoc()
  {
    tictoc(_algorithmPart.c_str());
  }

} // end namespace
