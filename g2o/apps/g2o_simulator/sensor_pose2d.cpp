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

#include "sensor_pose2d.h"

#include <cassert>
#include <unordered_set>
#include <utility>

namespace g2o {

SensorPose2D::SensorPose2D(std::string name)
    : BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2>(std::move(name)) {
  stepsToIgnore_ = 10;
}

bool SensorPose2D::isVisible(SensorPose2D::WorldObjectType* to) {
  if (!robotPoseVertex_) return false;

  assert(to && to->vertex());
  VertexType::EstimateType pose = to->vertex()->estimate();
  VertexType::EstimateType delta =
      robotPoseVertex_->estimate().inverse() * pose;
  Vector2 translation = delta.translation();
  double range2 = translation.squaredNorm();
  if (range2 > maxRange2_) return false;
  if (range2 < minRange2_) return false;
  translation.normalize();
  double bearing = acos(translation.x());
  if (fabs(bearing) > fov_) return false;
  if (fabs(delta.rotation().angle()) > maxAngularDifference_) return false;
  return true;
}

void SensorPose2D::addNoise(EdgeType* e) {
  EdgeType::ErrorVector noise = sampler_.generateSample();
  EdgeType::Measurement n(noise);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

void SensorPose2D::sense(BaseRobot& robot, World& world) {
  robotPoseVertex_ = robotPoseVertex<PoseVertexType>(robot, world);
  std::unordered_set<int> poses_to_ignore;
  int count = 0;
  for (auto it = robot.trajectory().rbegin();
       it != robot.trajectory().rend() && count < stepsToIgnore_; ++it) {
    poses_to_ignore.insert(*it);
    count++;
  }
  for (const auto& it : world.objects()) {
    auto* o = dynamic_cast<WorldObjectType*>(it.get());
    if (!o || !isVisible(o)) continue;
    if (poses_to_ignore.count(o->vertex()->id()) > 0) continue;
    auto e = mkEdge(o);
    if (!e) continue;
    e->setMeasurementFromState();
    addNoise(e.get());
    world.graph().addEdge(e);
  }
}

}  // namespace g2o
