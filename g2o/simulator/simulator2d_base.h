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

#ifndef G2O_SIMULATOR2D_BASE_H_
#define G2O_SIMULATOR2D_BASE_H_

#include "g2o/simulator/g2o_simulator_api.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d_addons/vertex_segment2d.h"
#include "simulator.h"

namespace g2o {

using WorldObjectSE2 = WorldObject<VertexSE2>;

using WorldObjectPointXY = WorldObject<VertexPointXY>;

using WorldObjectSegment2D = WorldObject<VertexSegment2D>;

using Robot2D = Robot<WorldObjectSE2>;

/**
 * @brief A 2D robot simulator of a robot moving in a 2D grid world.
 */
class G2O_SIMULATOR_API Simulator2D : public Simulator {
 public:
  /**
   * @brief Configuration of the 2D simulator
   */
  struct Config {
    double worldSize = 25.;
    int nlandmarks = 0;
    int simSteps = 100;
    bool hasOdom = false;
    // Poses and landmarks
    bool hasPoseSensor = false;
    bool hasPointSensor = false;
    bool hasCompass = false;
    bool hasGPS = false;
    bool hasPointBearingSensor = false;

    // Segment
    bool hasSegmentSensor = false;
    int nSegments = 0;
    int segmentGridSize = 50;
    double minSegmentLength = 0.5;
    double maxSegmentLength = 3.0;
  };

  Simulator2D() = default;
  explicit Simulator2D(Simulator2D::Config&& config);

  void setup() override;
  void simulate() override;

  Config config;
};

}  // namespace g2o

#endif
