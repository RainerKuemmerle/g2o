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

#include "g2o/core/eigen_types.h"

namespace g2o {

void EdgeSE2Segment2DPointLine::computeError() {
  const Vector3 prediction_vec = prediction();
  error_ = prediction_vec - measurement_;
  error_[2] = normalize_theta(error_[2]);
}

bool EdgeSE2Segment2DPointLine::setMeasurementFromState() {
  setMeasurement(prediction());
  return true;
}

[[nodiscard]] Vector3 EdgeSE2Segment2DPointLine::prediction() const {
  const VertexSE2* v1 = vertexXnRaw<0>();
  const VertexSegment2D* l2 = vertexXnRaw<1>();
  const SE2 iEst = v1->estimate().inverse();
  const Vector2 predP1 = iEst * l2->estimateP1();
  const Vector2 predP2 = iEst * l2->estimateP2();
  const Vector2 dP = predP2 - predP1;
  const Vector2 normal = Vector2(dP.y(), -dP.x()).normalized();
  Vector3 prediction;
  prediction << ((pointNum_ == 0) ? predP1 : predP2),
      std::atan2(normal.y(), normal.x());
  return prediction;
}

}  // namespace g2o
