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

#include "sensor_pose3d.h"

#include "g2o/types/slam3d/isometry3d_mappings.h"

namespace g2o {

SensorPose3D::SensorPose3D(const std::string& name)
    : BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3>(name) {
  stepsToIgnore_ = 10;
  information_.setIdentity();
  information_ *= 100;
  information_(3, 3) = 10000;
  information_(4, 4) = 10000;
  information_(5, 5) = 1000;
  setInformation(information_);
}

void SensorPose3D::addNoise(EdgeType* e) {
  EdgeType::ErrorVector noise = sampler_.generateSample();
  EdgeType::Measurement n = internal::fromVectorMQT(noise);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

bool SensorPose3D::isVisible(SensorPose3D::WorldObjectType* to) {
  if (!robotPoseObject_) return false;
  if (posesToIgnore_.find(to) != posesToIgnore_.end()) return false;

  assert(to && to->vertex());
  VertexType::EstimateType pose = to->vertex()->estimate();
  VertexType::EstimateType delta =
      robotPoseObject_->vertex()->estimate().inverse() * pose;
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

void SensorPose3D::sense() {
  robotPoseObject_ = nullptr;
  auto* r = dynamic_cast<RobotType*>(robot());
  auto it = r->trajectory().rbegin();
  posesToIgnore_.clear();
  int count = 0;
  while (it != r->trajectory().rend() && count < stepsToIgnore_) {
    if (!robotPoseObject_) robotPoseObject_ = *it;
    posesToIgnore_.insert(*it);
    ++it;
    count++;
  }
  for (auto* it : world()->objects()) {
    auto* o = dynamic_cast<WorldObjectType*>(it);
    if (o && isVisible(o)) {
      auto e = mkEdge(o);
      if (e && graph()) {
        graph()->addEdge(e);
        e->setMeasurementFromState();
        addNoise(e.get());
      }
    }
  }
}

}  // namespace g2o
