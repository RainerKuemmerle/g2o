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

#ifndef G2O_SENSOR_POSE3D_OFFSET_H_
#define G2O_SENSOR_POSE3D_OFFSET_H_

#include "g2o/simulator/simulator.h"
#include "g2o/types/slam3d/edge_se3_offset.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "pointsensorparameters.h"
#include "simulator3d_base.h"

namespace g2o {

class SensorPose3DOffset
    : public PointSensorParameters,
      public BinarySensor<Robot3D, EdgeSE3Offset, WorldObjectSE3> {
 public:
  explicit SensorPose3DOffset(std::string name);
  void sense(BaseRobot& robot, World& world) override;
  [[nodiscard]] int stepsToIgnore() const { return stepsToIgnore_; }
  void setStepsToIgnore(int stepsToIgnore) { stepsToIgnore_ = stepsToIgnore; }
  void addNoise(EdgeType* e) override;
  void addParameters(World& world) override;
  std::shared_ptr<ParameterSE3Offset> offsetParam1() { return offsetParam1_; };
  std::shared_ptr<ParameterSE3Offset> offsetParam2() { return offsetParam2_; };

 protected:
  bool isVisible(WorldObjectType* to);
  int stepsToIgnore_;
  std::shared_ptr<ParameterSE3Offset> offsetParam1_, offsetParam2_;
};

}  // namespace g2o

#endif
