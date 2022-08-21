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

#include "edge_se3_xyzprior.h"

namespace g2o {

EdgeSE3XYZPrior::EdgeSE3XYZPrior() {
  information().setIdentity();
  setMeasurement(Vector3::Zero());
  resizeParameters(1);
  installParameter<CacheSE3Offset::ParameterType>(0);
}

bool EdgeSE3XYZPrior::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  cache_ = resolveCache<CacheSE3Offset>(vertexXn<0>(), "CACHE_SE3_OFFSET", pv);
  return cache_ != nullptr;
}

bool EdgeSE3XYZPrior::read(std::istream& is) {
  readParamIds(is);
  internal::readVector(is, measurement_);
  return readInformationMatrix(is);
}

bool EdgeSE3XYZPrior::write(std::ostream& os) const {
  writeParamIds(os);
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE3XYZPrior::computeError() {
  const VertexSE3* v = vertexXnRaw<0>();
  error_ = v->estimate().translation() - measurement_;
}

void EdgeSE3XYZPrior::linearizeOplus() {
  const VertexSE3* v = vertexXnRaw<0>();
  jacobianOplusXi_.block<3, 3>(0, 0) = v->estimate().rotation();
  jacobianOplusXi_.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
}

bool EdgeSE3XYZPrior::setMeasurementFromState() {
  const VertexSE3* v = vertexXnRaw<0>();
  measurement_ = v->estimate().translation();
  return true;
}

void EdgeSE3XYZPrior::initialEstimate(
    const OptimizableGraph::VertexSet& /*from_*/,
    OptimizableGraph::Vertex* /*to_*/) {
  VertexSE3* v = vertexXnRaw<0>();
  assert(v && "Vertex for the Prior edge is not set");

  Isometry3 newEstimate = cache_->offsetParam()->offset().inverse() *
                          Eigen::Translation3d(measurement());
  if (information_.block<3, 3>(0, 0).array().abs().sum() ==
      0) {  // do not set translation, as that part of the information is all
            // zero
    newEstimate.translation() = v->estimate().translation();
  }
  v->setEstimate(newEstimate);
}

}  // namespace g2o
