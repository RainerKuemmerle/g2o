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

#include "edge_se2_pointxy.h"

namespace g2o {
namespace tutorial {

EdgeSE2PointXY::EdgeSE2PointXY() {
  resizeParameters(1);
  installParameter<CacheSE2Offset::ParameterType>(0);
}

bool EdgeSE2PointXY::read(std::istream& is) {
  readParamIds(is);
  internal::readVector(is, measurement_);
  return readInformationMatrix(is);
}

bool EdgeSE2PointXY::write(std::ostream& os) const {
  writeParamIds(os);
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE2PointXY::computeError() {
  const VertexPointXY* l2 = vertexXnRaw<1>();
  error_ = (sensorCache_->w2n() * l2->estimate()) - measurement_;
}

bool EdgeSE2PointXY::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  sensorCache_ = resolveCache<CacheSE2Offset>(vertexXn<0>(),
                                              "TUTORIAL_CACHE_SE2_OFFSET", pv);
  return sensorCache_ != nullptr;
}

}  // namespace tutorial
}  // namespace g2o
