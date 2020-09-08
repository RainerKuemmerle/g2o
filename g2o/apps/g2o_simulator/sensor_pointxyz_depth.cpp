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

namespace g2o {
using namespace std;
using namespace Eigen;

// SensorPointXYZDepth
SensorPointXYZDepth::SensorPointXYZDepth(const std::string& name_)
    : BinarySensor<Robot3D, EdgeSE3PointXYZDepth, WorldObjectTrackXYZ>(name_) {
  _offsetParam = 0;
  _information.setIdentity();
  _information *= 1000;
  setInformation(_information);
}

bool SensorPointXYZDepth::isVisible(SensorPointXYZDepth::WorldObjectType* to) {
  if (!_robotPoseObject) return false;
  assert(to && to->vertex());
  VertexType* v = to->vertex();
  VertexType::EstimateType pose = v->estimate();
  VertexType::EstimateType delta = _sensorPose.inverse() * pose;
  Vector3d translation = delta;
  double range2 = translation.squaredNorm();
  if (range2 > _maxRange2) return false;
  if (range2 < _minRange2) return false;
  translation.normalize();
  // the cameras have the z in front
  double bearing = acos(translation.z());
  if (fabs(bearing) > _fov) return false;
  return true;
}

void SensorPointXYZDepth::addParameters() {
  if (!_offsetParam) _offsetParam = new ParameterCamera();
  assert(world());
  world()->addParameter(_offsetParam);
}

void SensorPointXYZDepth::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = _sampler.generateSample();
  e->setMeasurement(e->measurement() + n);
  e->setInformation(information());
}

void SensorPointXYZDepth::sense() {
  if (!_offsetParam) return;
  _robotPoseObject = 0;
  RobotType* r = dynamic_cast<RobotType*>(robot());
  std::list<PoseObject*>::reverse_iterator it = r->trajectory().rbegin();
  int count = 0;
  while (it != r->trajectory().rend() && count < 1) {
    if (!_robotPoseObject) _robotPoseObject = *it;
    ++it;
    count++;
  }
  if (!_robotPoseObject) return;
  _sensorPose = _robotPoseObject->vertex()->estimate() * _offsetParam->offset();
  for (std::set<BaseWorldObject*>::iterator it = world()->objects().begin(); it != world()->objects().end(); ++it) {
    WorldObjectType* o = dynamic_cast<WorldObjectType*>(*it);
    if (o && isVisible(o)) {
      EdgeType* e = mkEdge(o);
      if (e && graph()) {
        e->setParameterId(0, _offsetParam->id());
        graph()->addEdge(e);
        e->setMeasurementFromState();
        addNoise(e);
      }
    }
  }
}
}  // namespace g2o
