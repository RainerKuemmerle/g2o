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

#include "sensor_se2_prior.h"

#include <cassert>

#include "g2o/core/eigen_types.h"

namespace g2o {

// SensorSE2Prior
SensorSE2Prior::SensorSE2Prior(std::string name)
    : UnarySensor<Robot2D, EdgeSE2Prior>(std::move(name)) {
  information_ = Vector3(100., 100., 1000.).asDiagonal();
  setInformation(information_);
}

void SensorSE2Prior::addNoise(EdgeType* e) {
  EdgeType::ErrorVector _n = sampler_.generateSample();
  EdgeType::Measurement n(_n);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

void SensorSE2Prior::sense(BaseRobot& robot, World& world) {
  robotPoseVertex_ = robotPoseVertex<PoseVertexType>(robot, world);
  if (!robotPoseVertex_) return;
  sensorPose_ = robotPoseVertex_->estimate();
  auto e = mkEdge();
  if (!e) return;
  world.graph().addEdge(e);
  e->setMeasurementFromState();
  addNoise(e.get());
}

}  // namespace g2o
