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

#include <cassert>
#include <utility>

namespace g2o {

SensorPointXYBearing::SensorPointXYBearing(std::string name)
    : BinarySensor<Robot2D, EdgeSE2PointXYBearing, WorldObjectPointXY>(
          std::move(name)) {
  information_(0, 0) = 180.0 / M_PI;
}

void SensorPointXYBearing::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = sampler_.generateSample();
  e->setMeasurement(e->measurement() + n(0));
  e->setInformation(information());
}

bool SensorPointXYBearing::isVisible(
    SensorPointXYBearing::WorldObjectType* to) {
  if (!robotPoseVertex_) return false;

  assert(to && to->vertex());
  const VertexType::EstimateType pose = to->vertex()->estimate();
  const VertexType::EstimateType delta =
      robotPoseVertex_->estimate().inverse() * pose;
  const double range2 = delta.squaredNorm();
  if (range2 > maxRange2_) return false;
  if (range2 < minRange2_) return false;
  double bearing = acos(delta.normalized().x());
  return std::abs(bearing) <= fov_;
}

void SensorPointXYBearing::sense(BaseRobot& robot, World& world) {
  robotPoseVertex_ = robotPoseVertex<PoseVertexType>(robot, world);
  for (const auto& it : world.objects()) {
    auto* o = dynamic_cast<WorldObjectType*>(it.get());
    if (!o || !isVisible(o)) continue;
    auto e = mkEdge(o);
    if (!e) continue;
    e->setMeasurementFromState();
    addNoise(e.get());
    world.graph().addEdge(e);
  }
}

}  // namespace g2o
