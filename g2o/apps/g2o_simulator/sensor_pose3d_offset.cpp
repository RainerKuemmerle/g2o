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

#include "sensor_pose3d_offset.h"

#include <cassert>
#include <unordered_set>
#include <utility>

#include "g2o/apps/g2o_simulator/simulator.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"

namespace g2o {

SensorPose3DOffset::SensorPose3DOffset(std::string name)
    : BinarySensor<Robot3D, EdgeSE3Offset, WorldObjectSE3>(std::move(name)) {
  offsetParam1_ = offsetParam2_ = nullptr;
  stepsToIgnore_ = 10;
  information_.setIdentity();
  information_ *= 100;
  information_(3, 3) = 10000;
  information_(4, 4) = 10000;
  information_(5, 5) = 1000;
  setInformation(information_);
}

void SensorPose3DOffset::addParameters(World& world) {
  if (!offsetParam1_) offsetParam1_ = std::make_shared<ParameterSE3Offset>();
  if (!offsetParam2_) offsetParam2_ = std::make_shared<ParameterSE3Offset>();
  world.addParameter(offsetParam1_);
  world.addParameter(offsetParam2_);
}

void SensorPose3DOffset::addNoise(EdgeType* e) {
  EdgeType::ErrorVector noise = sampler_.generateSample();
  EdgeType::Measurement n = internal::fromVectorMQT(noise);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

bool SensorPose3DOffset::isVisible(SensorPose3DOffset::WorldObjectType* to) {
  if (!robotPoseVertex_) return false;
  assert(to && to->vertex());
  VertexType::EstimateType pose = to->vertex()->estimate();
  VertexType::EstimateType delta =
      robotPoseVertex_->estimate().inverse() * pose;
  Vector3 translation = delta.translation();
  double range2 = translation.squaredNorm();
  if (range2 > maxRange2_) return false;
  if (range2 < minRange2_) return false;
  translation.normalize();
  double bearing = acos(translation.x());
  if (fabs(bearing) > fov_) return false;
  AngleAxis a(delta.rotation());
  return fabs(a.angle()) <= maxAngularDifference_;
}

void SensorPose3DOffset::sense(BaseRobot& robot, World& world) {
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
    e->setParameterId(0, offsetParam1_->id());
    e->setParameterId(1, offsetParam2_->id());
    world.graph().addEdge(e);
    e->setMeasurementFromState();
    addNoise(e.get());
  }
}

}  // namespace g2o
