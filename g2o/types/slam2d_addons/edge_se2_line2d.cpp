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

#include "edge_se2_line2d.h"

namespace g2o {

bool EdgeSE2Line2D::read(std::istream& is) {
  internal::readVector(is, measurement_);
  return readInformationMatrix(is);
}

bool EdgeSE2Line2D::write(std::ostream& os) const {
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE2Line2D::initialEstimate(const OptimizableGraph::VertexSet& from,
                                    OptimizableGraph::Vertex* to) {
  assert(from.size() == 1 && from.count(vertices_[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexLine2D");

  auto vi = vertexXn<0>();
  VertexLine2D* vj = vertexXnRaw<1>();
  if (from.count(vi) > 0 && to == vj) {
    SE2 T = vi->estimate();
    Vector2 est = measurement_;
    est[0] += T.rotation().angle();
    est[0] = normalize_theta(est[0]);
    Vector2 n(std::cos(est[0]), std::sin(est[0]));
    est[1] += n.dot(T.translation());
    vj->setEstimate(Line2D(est));
  }
}

}  // namespace g2o
