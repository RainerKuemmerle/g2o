// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "edge_se2_segment2d_pointLine.h"

#include <cmath>

namespace g2o {

void EdgeSE2Segment2DPointLine::computeError() {
  const VertexSE2* v1 = vertexXnRaw<0>();
  const VertexSegment2D* l2 = vertexXnRaw<1>();
  SE2 iEst = v1->estimate().inverse();
  Vector2 predP1 = iEst * l2->estimateP1();
  Vector2 predP2 = iEst * l2->estimateP2();
  Vector2 dP = predP2 - predP1;
  Vector2 normal(dP.y(), -dP.x());
  normal.normalize();
  Vector3 prediction;
  prediction[2] = std::atan2(normal.y(), normal.x());
  Eigen::Map<Vector2> pt(prediction.data());
  pt = (pointNum_ == 0) ? predP1 : predP2;
  error_ = prediction - measurement_;
  error_[2] = normalize_theta(error_[2]);
}

bool EdgeSE2Segment2DPointLine::setMeasurementFromState() {
  const VertexSE2* v1 = vertexXnRaw<0>();
  const VertexSegment2D* l2 = vertexXnRaw<1>();
  SE2 iEst = v1->estimate().inverse();
  Vector2 predP1 = iEst * l2->estimateP1();
  Vector2 predP2 = iEst * l2->estimateP2();
  Vector2 dP = predP2 - predP1;
  Vector2 normal(dP.y(), -dP.x());
  normal.normalize();
  Vector3 prediction;
  prediction[2] = std::atan2(normal.y(), normal.x());
  Eigen::Map<Vector2> pt(prediction.data());
  pt = (pointNum_ == 0) ? predP1 : predP2;
  setMeasurement(prediction);
  return true;
}

}  // namespace g2o
