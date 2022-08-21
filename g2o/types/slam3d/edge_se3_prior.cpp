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

#include "edge_se3_prior.h"

#include <iostream>

#include "isometry3d_gradients.h"

namespace g2o {

// point to camera projection, monocular
EdgeSE3Prior::EdgeSE3Prior() {
  setMeasurement(Isometry3::Identity());
  information().setIdentity();
  resizeParameters(1);
  installParameter<CacheSE3Offset::ParameterType>(0);
}

bool EdgeSE3Prior::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  cache_ = resolveCache<CacheSE3Offset>(vertexXn<0>(), "CACHE_SE3_OFFSET", pv);
  return cache_ != nullptr;
}

bool EdgeSE3Prior::read(std::istream& is) {
  bool state = readParamIds(is);
  Vector7 meas;
  state &= internal::readVector(is, meas);
  setMeasurement(internal::fromVectorQT(meas));
  state &= readInformationMatrix(is);
  return state;
}

bool EdgeSE3Prior::write(std::ostream& os) const {
  writeParamIds(os);
  internal::writeVector(os, internal::toVectorQT(measurement()));
  writeInformationMatrix(os);
  return os.good();
}

void EdgeSE3Prior::computeError() {
  Isometry3 delta = inverseMeasurement_ * cache_->n2w();
  error_ = internal::toVectorMQT(delta);
}

void EdgeSE3Prior::linearizeOplus() {
  VertexSE3* from = vertexXnRaw<0>();
  Isometry3 E;
  Isometry3 Z;
  Isometry3 X;
  Isometry3 P;
  X = from->estimate();
  P = cache_->offsetParam()->offset();
  Z = measurement_;
  internal::computeEdgeSE3PriorGradient(E, jacobianOplusXi_, Z, X, P);
}

bool EdgeSE3Prior::setMeasurementFromState() {
  setMeasurement(cache_->n2w());
  return true;
}

void EdgeSE3Prior::initialEstimate(const OptimizableGraph::VertexSet& /*from_*/,
                                   OptimizableGraph::Vertex* /*to_*/) {
  VertexSE3* v = vertexXnRaw<0>();
  assert(v && "Vertex for the Prior edge is not set");

  Isometry3 newEstimate =
      cache_->offsetParam()->offset().inverse() * measurement();
  // do not set translation, as that part of the information is all zero
  if (information_.block<3, 3>(0, 0).array().abs().sum() == 0) {
    newEstimate.translation() = v->estimate().translation();
  }
  // do not set rotation, as that part of the information is all zero
  if (information_.block<3, 3>(3, 3).array().abs().sum() == 0) {
    newEstimate.matrix().block<3, 3>(0, 0) =
        internal::extractRotation(v->estimate());
  }
  v->setEstimate(newEstimate);
}

}  // namespace g2o
