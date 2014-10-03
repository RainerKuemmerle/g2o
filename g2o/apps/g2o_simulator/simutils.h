// g2o - General Graph Optimization
// Copyright (C) 2011 G. Grisetti, R. Kuemmerle, W. Burgard
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

#ifndef G2O_SIMUTILS_H_
#define G2O_SIMUTILS_H_

#include "simulator2d_base.h"
#include "pointsensorparameters.h"
#include "g2o_simulator_api.h"

namespace g2o {
  // -1: outside
  // 0: p1Clipped
  // 1: p2clipped
  // 2: inside
  // 3: all clipped

  int clipSegmentCircle(Eigen::Vector2d& p1, Eigen::Vector2d& p2, double r);

  // -1: outside
  // 0: p1Clipped
  // 1: p2clipped
  // 2: inside
  int clipSegmentLine(Eigen::Vector2d& p1, Eigen::Vector2d& p2, double a, double b, double c );
  

  // -1: outside
  // 0: p1Clipped
  // 1: p2clipped
  // 2: inside
  // 3: all clipped
  int clipSegmentFov(Eigen::Vector2d& p1, Eigen::Vector2d& p2, double min, double max);

  
  Eigen::Vector2d computeLineParameters(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
}

#endif
