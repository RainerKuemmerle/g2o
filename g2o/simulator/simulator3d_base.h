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

#ifndef G2O_SIMULATOR3D_BASE_H_
#define G2O_SIMULATOR3D_BASE_H_

#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d_addons/vertex_line3d.h"
#include "simulator.h"

namespace g2o {

using WorldObjectSE3 = WorldObject<VertexSE3>;

using WorldObjectTrackXYZ = WorldObject<VertexPointXYZ>;

using WorldObjectLine3D = WorldObject<VertexLine3D>;

using Robot3D = Robot<WorldObjectSE3>;

/**
 * @brief A 3D robot simulator of a robot moving in a grid world.
 */
class G2O_SIMULATOR_API Simulator3D : public Simulator {
 public:
  /**
   * @brief Configuration of the 3D simulator
   */
  struct Config : public Simulator::Config {
    bool hasPointDepthSensor = false;
    bool hasPointDisparitySensor = false;
  };

  Simulator3D() = default;
  explicit Simulator3D(Simulator3D::Config&& config);

  void setup() override;
  void simulate() override;

  Config config;
};

}  // namespace g2o

#endif
