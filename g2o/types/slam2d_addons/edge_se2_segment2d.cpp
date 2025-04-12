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

namespace g2o {

EdgeSE2Segment2D::EdgeSE2Segment2D()
    : BaseBinaryEdge<4, Vector4, VertexSE2, VertexSegment2D>() {}

bool EdgeSE2Segment2D::read(std::istream& is) {
  internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeSE2Segment2D::write(std::ostream& os) const {
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE2Segment2D::initialEstimate(const OptimizableGraph::VertexSet& from,
                                       OptimizableGraph::Vertex* to) {
  assert(from.size() == 1 && from.count(_vertices[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexSegment2D. I could if "
         "i wanted. Not now");

  VertexSE2* vi = static_cast<VertexSE2*>(_vertices[0]);
  VertexSegment2D* vj = static_cast<VertexSegment2D*>(_vertices[1]);
  if (from.count(vi) > 0 && to == vj) {
    vj->setEstimateP1(vi->estimate() * measurementP1());
    vj->setEstimateP2(vi->estimate() * measurementP2());
  }
}

}  // namespace g2o
