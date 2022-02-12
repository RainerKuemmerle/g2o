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

#include "edge_se2_prior.h"

namespace g2o {

void EdgeSE2Prior::initialEstimate(const OptimizableGraph::VertexSet& from,
                                   OptimizableGraph::Vertex* to) {
  assert(from.empty());
  (void)from;
  (void)to;
  VertexSE2* v1 = vertexXnRaw<0>();
  v1->setEstimate(measurement_);
}

bool EdgeSE2Prior::read(std::istream& is) {
  Vector3 p;
  internal::readVector(is, p);
  setMeasurement(SE2(p));
  inverseMeasurement_ = measurement_.inverse();
  readInformationMatrix(is);
  return true;
}

bool EdgeSE2Prior::write(std::ostream& os) const {
  internal::writeVector(os, measurement().toVector());
  return writeInformationMatrix(os);
}

void EdgeSE2Prior::setMeasurement(const SE2& m) {
  measurement_ = m;
  inverseMeasurement_ = m.inverse();
}

bool EdgeSE2Prior::setMeasurementData(const number_t* d) {
  measurement_ = SE2(d[0], d[1], d[2]);
  inverseMeasurement_ = measurement_.inverse();
  return true;
}

}  // namespace g2o
