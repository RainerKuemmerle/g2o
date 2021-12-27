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

#include "sensor_pointxy_bearing.h"

namespace g2o {

SensorPointXYBearing::SensorPointXYBearing(const std::string& name)
    : BinarySensor<Robot2D, EdgeSE2PointXYBearing, WorldObjectPointXY>(name) {
  information_(0, 0) = 180.0 / M_PI;
}

void SensorPointXYBearing::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = sampler_.generateSample();
  e->setMeasurement(e->measurement() + n(0));
  e->setInformation(information());
}

bool SensorPointXYBearing::isVisible(
    SensorPointXYBearing::WorldObjectType* to) {
  if (!robotPoseObject_) return false;

  assert(to && to->vertex());
  VertexType::EstimateType pose = to->vertex()->estimate();
  VertexType::EstimateType delta =
      robotPoseObject_->vertex()->estimate().inverse() * pose;
  Vector2 translation = delta;
  double range2 = translation.squaredNorm();
  if (range2 > maxRange2_) return false;
  if (range2 < minRange2_) return false;
  translation.normalize();
  double bearing = acos(translation.x());
  return fabs(bearing) <= fov_;
}

void SensorPointXYBearing::sense() {
  robotPoseObject_ = nullptr;
  auto* r = dynamic_cast<RobotType*>(robot());
  auto it = r->trajectory().rbegin();
  int count = 0;
  while (it != r->trajectory().rend() && count < 1) {
    if (!robotPoseObject_) robotPoseObject_ = *it;
    ++it;
    count++;
  }
  for (auto* it : world()->objects()) {
    auto* o = dynamic_cast<WorldObjectType*>(it);
    if (o && isVisible(o)) {
      auto e = mkEdge(o);
      if (e && graph()) {
        e->setMeasurementFromState();
        addNoise(e.get());
        graph()->addEdge(e);
      }
    }
  }
}

}  // namespace g2o
