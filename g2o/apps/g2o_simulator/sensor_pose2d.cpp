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

using namespace Eigen;

namespace g2o{
  SensorPose2D::SensorPose2D(const std::string& name_):
    BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2>(name_)
  {
    _stepsToIgnore = 10;
  }

  bool SensorPose2D::isVisible(SensorPose2D::WorldObjectType* to){
    if (! _robotPoseObject)
      return false;
    if (_posesToIgnore.find(to)!=_posesToIgnore.end())
      return false;
    
    assert(to && to->vertex());
    VertexType* v=to->vertex();
    VertexType::EstimateType pose=v->estimate();
    VertexType::EstimateType delta = _robotPoseObject->vertex()->estimate().inverse()*pose;
    Vector2d translation=delta.translation();
    double range2=translation.squaredNorm();
    if (range2>_maxRange2)
      return false;
    if (range2<_minRange2)
      return false;
    translation.normalize();
    double bearing=acos(translation.x());
    if (fabs(bearing)>_fov)
      return false;
    if (fabs(delta.rotation().angle())>_maxAngularDifference)
      return false;
    return true;
  }
  

  void SensorPose2D::addNoise(EdgeType* e){
    EdgeType::ErrorVector noise=_sampler.generateSample();
    EdgeType::Measurement n;
    n.fromVector(noise);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }
 
  void SensorPose2D::sense() {
    _robotPoseObject=0;
    RobotType* r= dynamic_cast<RobotType*>(robot());
    std::list<PoseObject*>::reverse_iterator it=r->trajectory().rbegin();
    _posesToIgnore.clear();
    int count = 0;
    while (it!=r->trajectory().rend() && count < _stepsToIgnore){
      if (!_robotPoseObject)
  _robotPoseObject = *it;
      _posesToIgnore.insert(*it);
      it++;
      count++;
    }
    for (std::set<BaseWorldObject*>::iterator it=world()->objects().begin();
   it!=world()->objects().end(); it++){
      WorldObjectType* o=dynamic_cast<WorldObjectType*>(*it);
      if (o && isVisible(o)){
	EdgeType* e=mkEdge(o);  
	if (e && graph()) {
	  e->setMeasurementFromState();
	  addNoise(e);
	  graph()->addEdge(e);
	}
      }
    }
  }

} // end namespace
