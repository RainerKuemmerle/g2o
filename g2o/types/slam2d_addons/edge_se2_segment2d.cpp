// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include "edge_se2_segment2d.h"

#include <cassert>
#include <memory>

namespace g2o {

void EdgeSE2Segment2D::computeError() {
  const VertexSE2* v1 = vertexXnRaw<0>();
  const VertexSegment2D* l2 = vertexXnRaw<1>();
  Eigen::Map<Vector2> error1(&error_(0));
  Eigen::Map<Vector2> error2(&error_(2));
  SE2 iEst = v1->estimate().inverse();
  error1 = (iEst * l2->estimateP1());
  error2 = (iEst * l2->estimateP2());
  error_ = error_ - measurement_;
}

bool EdgeSE2Segment2D::setMeasurementFromState() {
  const VertexSE2* v1 = vertexXnRaw<0>();
  const VertexSegment2D* l2 = vertexXnRaw<1>();
  SE2 iEst = v1->estimate().inverse();
  setMeasurementP1(iEst * l2->estimateP1());
  setMeasurementP2(iEst * l2->estimateP2());
  return true;
}

void EdgeSE2Segment2D::initialEstimate(const OptimizableGraph::VertexSet& from,
                                       OptimizableGraph::Vertex* to) {
  assert(from.size() == 1 && from.count(vertices_[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexSegment2D. I could if "
         "i wanted. Not now");

  auto vi = vertexXn<0>();
  auto vj = vertexXn<1>();
  if (from.count(vi) > 0 && to == vj.get()) {
    vj->setEstimateP1(vi->estimate() * measurementP1());
    vj->setEstimateP2(vi->estimate() * measurementP2());
  }
}

}  // namespace g2o
