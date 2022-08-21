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

#include "edge_se3_offset.h"

#include <iostream>

#include "isometry3d_gradients.h"
#include "parameter_se3_offset.h"

namespace g2o {

EdgeSE3Offset::EdgeSE3Offset() {
  information().setIdentity();
  resizeParameters(2);
  installParameter<CacheSE3Offset::ParameterType>(0);
  installParameter<CacheSE3Offset::ParameterType>(1);
}

bool EdgeSE3Offset::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  cacheFrom_ =
      resolveCache<CacheSE3Offset>(vertexXn<0>(), "CACHE_SE3_OFFSET", pv);
  pv[0] = parameters_[1];
  cacheTo_ =
      resolveCache<CacheSE3Offset>(vertexXn<0>(), "CACHE_SE3_OFFSET", pv);
  return (cacheFrom_ && cacheTo_);
}

bool EdgeSE3Offset::read(std::istream& is) {
  bool state = readParamIds(is);

  Vector7 meas;
  state &= internal::readVector(is, meas);
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4::MapType(meas.data() + 3).normalize();
  setMeasurement(internal::fromVectorQT(meas));

  state &= readInformationMatrix(is);
  return state;
}

bool EdgeSE3Offset::write(std::ostream& os) const {
  writeParamIds(os);
  internal::writeVector(os, internal::toVectorQT(measurement_));
  writeInformationMatrix(os);
  return os.good();
}

void EdgeSE3Offset::computeError() {
  Isometry3 delta = inverseMeasurement_ * cacheFrom_->w2n() * cacheTo_->n2w();
  error_ = internal::toVectorMQT(delta);
}

bool EdgeSE3Offset::setMeasurementFromState() {
  Isometry3 delta = cacheFrom_->w2n() * cacheTo_->n2w();
  setMeasurement(delta);
  return true;
}

void EdgeSE3Offset::linearizeOplus() {
  // BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>::linearizeOplus();

  VertexSE3* from = vertexXnRaw<0>();
  VertexSE3* to = vertexXnRaw<1>();
  Isometry3 E;
  const Isometry3& Xi = from->estimate();
  const Isometry3& Xj = to->estimate();
  const Isometry3& Pi = cacheFrom_->offsetParam()->offset();
  const Isometry3& Pj = cacheTo_->offsetParam()->offset();
  const Isometry3& Z = measurement_;
  internal::computeEdgeSE3Gradient(E, jacobianOplusXi_, jacobianOplusXj_, Z, Xi,
                                   Xj, Pi, Pj);
}

void EdgeSE3Offset::initialEstimate(const OptimizableGraph::VertexSet& from_,
                                    OptimizableGraph::Vertex* /*to_*/) {
  VertexSE3* from = vertexXnRaw<0>();
  VertexSE3* to = vertexXnRaw<1>();

  Isometry3 virtualMeasurement = cacheFrom_->offsetParam()->offset() *
                                 measurement() *
                                 cacheTo_->offsetParam()->offset().inverse();

  if (from_.count(vertexXn<0>()) > 0) {
    to->setEstimate(from->estimate() * virtualMeasurement);
  } else
    from->setEstimate(to->estimate() * virtualMeasurement.inverse());
}

}  // namespace g2o
