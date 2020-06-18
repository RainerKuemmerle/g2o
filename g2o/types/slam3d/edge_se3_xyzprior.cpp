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

  EdgeSE3XYZPrior::EdgeSE3XYZPrior() : BaseUnaryEdge<3, Vector3, g2o::VertexSE3>()
  {
    information().setIdentity();
    setMeasurement(Vector3::Zero());
    _cache = 0;
    _offsetParam = 0;
    resizeParameters(1);
    installParameter(_offsetParam, 0);
  }

  bool EdgeSE3XYZPrior::resolveCaches(){
    assert(_offsetParam);
    ParameterVector pv(1);
    pv[0] = _offsetParam;
    resolveCache(_cache, (OptimizableGraph::Vertex*)_vertices[0], "CACHE_SE3_OFFSET", pv);
    return _cache != 0;
  }

  bool EdgeSE3XYZPrior::read(std::istream& is)
  {
    readParamIds(is);
    internal::readVector(is, _measurement);
    return readInformationMatrix(is);
  }

  bool EdgeSE3XYZPrior::write(std::ostream& os) const {
    writeParamIds(os);
    internal::writeVector(os, measurement());
    return writeInformationMatrix(os);
  }

  void EdgeSE3XYZPrior::computeError() {
    const VertexSE3* v = static_cast<const VertexSE3*>(_vertices[0]);
    _error = v->estimate().translation() - _measurement;
  }

  void EdgeSE3XYZPrior::linearizeOplus() {
    _jacobianOplusXi << Matrix3::Identity();
  }

  bool EdgeSE3XYZPrior::setMeasurementFromState() {
    const VertexSE3* v = static_cast<const VertexSE3*>(_vertices[0]);
    _measurement = v->estimate().translation();
    return true;
  }

  void EdgeSE3XYZPrior::initialEstimate(const OptimizableGraph::VertexSet& /*from_*/, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3 *v = static_cast<VertexSE3*>(_vertices[0]);
    assert(v && "Vertex for the Prior edge is not set");

    Isometry3 newEstimate = _offsetParam->offset().inverse() * Eigen::Translation3d(measurement());
    if (_information.block<3,3>(0,0).array().abs().sum() == 0){ // do not set translation, as that part of the information is all zero
      newEstimate.translation() = v->estimate().translation();
    }
    v->setEstimate(newEstimate);
  }

} // end namespace
