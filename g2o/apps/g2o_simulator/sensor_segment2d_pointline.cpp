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

#include "sensor_segment2d_pointline.h"
#include  "g2o/apps/g2o_simulator/simutils.h"
using namespace std;
using namespace Eigen;

namespace g2o{

  SensorSegment2DPointLine::SensorSegment2DPointLine(const std::string& name_): BinarySensor<Robot2D, EdgeSE2Segment2DPointLine, WorldObjectSegment2D>(name_) {}

  void SensorSegment2DPointLine::addNoise(EdgeType* e){
    EdgeType::ErrorVector n=_sampler.generateSample();
    e->setMeasurement(e->measurement()+n);
    e->setInformation(information());
  }

  bool SensorSegment2DPointLine::isVisible(SensorSegment2DPointLine::WorldObjectType* to){
    if (! _robotPoseObject)
      return false;

    assert(to && to->vertex());
    VertexType* v=to->vertex();

    Vector2d p1, p2;
    SE2 iRobot=_robotPoseObject->vertex()->estimate().inverse();
    p1 = iRobot * v->estimateP1();
    p2 = iRobot * v->estimateP2();

    Vector3d vp1(p1.x(), p1.y(), 0.);
    Vector3d vp2(p2.x(), p2.y(), 0.);
    Vector3d cp=vp1.cross(vp2); // visibility check
    if (cp[2]<0)
      return false;

    int circleClip = clipSegmentCircle(p1,p2,sqrt(_maxRange2));
    bool clip1=false, clip2=false;
    switch(circleClip){
      case -1:
        return false;
      case  0:
        clip1 = true;
        break;
      case  1:
        clip2 = true;
        break;
      case  3:
        clip1 = true;
        clip2 = true;
        break;
      default:;
    }

    int fovClip=clipSegmentFov(p1,p2,-_fov, +_fov);
    switch(fovClip){
      case -1:
        return false;
      case  0:
        clip1 = true;
        break;
      case  1:
        clip2 = true;
        break;
      case  3:
        clip1 = true;
        clip2 = true;
        break;
      default:;
    }
    if ((clip1 && !clip2)) {
      _visiblePoint = 1;
      return true;
    }
    if ((!clip1 && clip2)) {
      _visiblePoint = 0;
      return true;
    }
    return false;
  }


  void SensorSegment2DPointLine::sense() {
    _robotPoseObject=0;
    RobotType* r= dynamic_cast<RobotType*>(robot());
    std::list<PoseObject*>::reverse_iterator it=r->trajectory().rbegin();
    int count = 0;
    while (it!=r->trajectory().rend() && count < 1){
      if (!_robotPoseObject)
        _robotPoseObject = *it;
      ++it;
      count++;
    }
    for (std::set<BaseWorldObject*>::iterator it=world()->objects().begin(); it!=world()->objects().end(); ++it){
      WorldObjectType* o=dynamic_cast<WorldObjectType*>(*it);
      if (o && isVisible(o)){
        EdgeType* e=mkEdge(o);
        if (e && graph()) {
          e->setPointNum(_visiblePoint);
          e->setMeasurementFromState();
          addNoise(e);
          graph()->addEdge(e);
        }
      }
    }
  }

} // end namespace
