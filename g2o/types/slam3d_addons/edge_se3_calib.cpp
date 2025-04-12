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

#include "edge_se3_calib.h"

#include "g2o/types/slam3d/vertex_se3.h"

namespace g2o {

EdgeSE3Calib::EdgeSE3Calib() : BaseVariableSizedEdge<6, Isometry3>() {
  resize(3);
}

void EdgeSE3Calib::computeError() {
  const VertexSE3* v1 = static_cast<const VertexSE3*>(_vertices[0]);
  const VertexSE3* v2 = static_cast<const VertexSE3*>(_vertices[1]);
  const VertexSE3* calib = static_cast<const VertexSE3*>(_vertices[2]);
  _error = g2o::internal::toVectorMQT(
      _measurement.inverse() * calib->estimate().inverse() *
      v1->estimate().inverse() * v2->estimate() * calib->estimate());
}

bool EdgeSE3Calib::write(std::ostream& os) const {
  internal::writeVector(os, internal::toVectorQT(_measurement));
  return writeInformationMatrix(os);
}

bool EdgeSE3Calib::read(std::istream& is) {
  Vector7 meas;
  internal::readVector(is, meas);
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4::MapType(meas.data() + 3).normalize();
  setMeasurement(g2o::internal::fromVectorQT(meas));
  return readInformationMatrix(is);
}

}  // namespace g2o
