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

#include "sensor_pointxyz_depth.h"

#include <cassert>

#include "g2o/simulator/simulator.h"

namespace g2o {

// SensorPointXYZDepth
SensorPointXYZDepth::SensorPointXYZDepth(const std::string& name)
    : BinarySensor<Robot3D, EdgeSE3PointXYZDepth, WorldObjectTrackXYZ>(name) {
  offsetParam_ = nullptr;
  information_.setIdentity();
  information_ *= 1000;
  setInformation(information_);
}

bool SensorPointXYZDepth::isVisible(SensorPointXYZDepth::WorldObjectType* to) {
  if (!robotPoseVertex_) return false;
  assert(to && to->vertex());
  VertexType::EstimateType pose = to->vertex()->estimate();
  VertexType::EstimateType delta = sensorPose_.inverse() * pose;
  Vector3 translation = delta;
  double range2 = translation.squaredNorm();
  if (range2 > maxRange2_) return false;
  if (range2 < minRange2_) return false;
  translation.normalize();
  // the cameras have the z in front
  double bearing = acos(translation.z());
  return fabs(bearing) <= fov_;
}

void SensorPointXYZDepth::addParameters(World& world) {
  if (!offsetParam_) offsetParam_ = std::make_shared<ParameterCamera>();
  world.addParameter(offsetParam_);
}

void SensorPointXYZDepth::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = sampler_.generateSample();
  e->setMeasurement(e->measurement() + n);
  e->setInformation(information());
}

void SensorPointXYZDepth::sense(BaseRobot& robot, World& world) {
  if (!offsetParam_) return;
  robotPoseVertex_ = robotPoseVertex<PoseVertexType>(robot, world);

  if (!robotPoseVertex_) return;
  sensorPose_ = robotPoseVertex_->estimate() * offsetParam_->param().offset();
  for (const auto& it : world.objects()) {
    auto* o = dynamic_cast<WorldObjectType*>(it.get());
    if (!o || !isVisible(o)) continue;
    auto e = mkEdge(o);
    if (!e) continue;
    e->setParameterId(0, offsetParam_->id());
    world.graph().addEdge(e);
    e->setMeasurementFromState();
  }
}

}  // namespace g2o
