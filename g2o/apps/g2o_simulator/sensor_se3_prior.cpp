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

#include "sensor_se3_prior.h"

namespace g2o {

// SensorSE3Prior
SensorSE3Prior::SensorSE3Prior(const std::string& name)
    : UnarySensor<Robot3D, EdgeSE3Prior>(name) {
  offsetParam_ = nullptr;
  information_.setIdentity();
  information_ *= 1000;
  information_(2, 2) = 10;
  setInformation(information_);
}

void SensorSE3Prior::addParameters() {
  if (!offsetParam_) offsetParam_ = std::make_shared<ParameterSE3Offset>();
  assert(world());
  world()->addParameter(offsetParam_);
}

void SensorSE3Prior::addNoise(EdgeType* e) {
  EdgeType::ErrorVector _n = sampler_.generateSample();
  EdgeType::Measurement n = internal::fromVectorMQT(_n);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

void SensorSE3Prior::sense() {
  if (!offsetParam_) {
    return;
  }
  robotPoseObject_ = nullptr;
  auto* r = dynamic_cast<RobotType*>(robot());
  auto it = r->trajectory().rbegin();
  int count = 0;
  while (it != r->trajectory().rend() && count < 1) {
    if (!robotPoseObject_) robotPoseObject_ = *it;
    ++it;
    count++;
  }
  if (!robotPoseObject_) return;
  sensorPose_ = robotPoseObject_->vertex()->estimate() * offsetParam_->offset();
  auto e = mkEdge();
  if (e && graph()) {
    e->setParameterId(0, offsetParam_->id());
    graph()->addEdge(e);
    e->setMeasurementFromState();
    addNoise(e.get());
  }
}

}  // namespace g2o
