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

#include <cassert>
#include <utility>

#include "g2o/simulator/simutils.h"

namespace g2o {

SensorSegment2DPointLine::SensorSegment2DPointLine(std::string name)
    : BinarySensor<Robot2D, EdgeSE2Segment2DPointLine, WorldObjectSegment2D>(
          std::move(name)) {}

void SensorSegment2DPointLine::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = sampler_.generateSample();
  e->setMeasurement(e->measurement() + n);
  e->setInformation(information());
}

bool SensorSegment2DPointLine::isVisible(
    SensorSegment2DPointLine::WorldObjectType* to) {
  if (!robotPoseVertex_) return false;

  assert(to && to->vertex());
  VertexType* v = to->vertex().get();

  const SE2 iRobot = robotPoseVertex_->estimate().inverse();
  Vector2 p1 = iRobot * v->estimateP1();
  Vector2 p2 = iRobot * v->estimateP2();

  Vector3 vp1(p1.x(), p1.y(), 0.);
  Vector3 vp2(p2.x(), p2.y(), 0.);
  Vector3 cp = vp1.cross(vp2);  // visibility check
  if (cp[2] < 0) return false;

  int circleClip = clipSegmentCircle(p1, p2, sqrt(maxRange2_));
  bool clip1 = false;
  bool clip2 = false;
  switch (circleClip) {
    case -1:
      return false;
    case 0:
      clip1 = true;
      break;
    case 1:
      clip2 = true;
      break;
    case 3:
      clip1 = true;
      clip2 = true;
      break;
    default:;
  }

  int fovClip = clipSegmentFov(p1, p2, -fov_, +fov_);
  switch (fovClip) {
    case -1:
      return false;
    case 0:
      clip1 = true;
      break;
    case 1:
      clip2 = true;
      break;
    case 3:
      clip1 = true;
      clip2 = true;
      break;
    default:;
  }
  if ((clip1 && !clip2)) {
    visiblePoint_ = 1;
    return true;
  }
  if ((!clip1 && clip2)) {
    visiblePoint_ = 0;
    return true;
  }
  return false;
}

void SensorSegment2DPointLine::sense(BaseRobot& robot, World& world) {
  robotPoseVertex_ = robotPoseVertex<PoseVertexType>(robot, world);
  for (const auto& it : world.objects()) {
    auto* o = dynamic_cast<WorldObjectType*>(it.get());
    if (!o || isVisible(o)) continue;
    auto e = mkEdge(o);
    if (!e) continue;
    e->setPointNum(visiblePoint_);
    e->setMeasurementFromState();
    addNoise(e.get());
    world.graph().addEdge(e);
  }
}

}  // namespace g2o
