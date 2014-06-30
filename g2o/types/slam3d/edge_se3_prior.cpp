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
#include "isometry3d_gradients.h"
#include <iostream>

namespace g2o {
  using namespace std;

  // point to camera projection, monocular
  EdgeSE3Prior::EdgeSE3Prior() : BaseUnaryEdge<6, Eigen::Isometry3d, VertexSE3>() {
    setMeasurement(Eigen::Isometry3d::Identity());
    information().setIdentity();
    _cache = 0;
    _offsetParam = 0;
    resizeParameters(1);
    installParameter(_offsetParam, 0);
  }


  bool EdgeSE3Prior::resolveCaches(){
    assert(_offsetParam);
    ParameterVector pv(1);
    pv[0]=_offsetParam;
    resolveCache(_cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE3_OFFSET",pv);
    return _cache != 0;
  }



  bool EdgeSE3Prior::read(std::istream& is) {
    int pid;
    is >> pid;
    if (!setParameterId(0, pid))
      return false;
    // measured keypoint
    Vector7d meas;
    for (int i=0; i<7; i++) is >> meas[i];
    setMeasurement(internal::fromVectorQT(meas));
    // don't need this if we don't use it in error calculation (???)
    // information matrix is the identity for features, could be changed to allow arbitrary covariances    
    if (is.bad()) {
      return false;
    }
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix
      information().setIdentity();
    } 
    return true;
  }

  bool EdgeSE3Prior::write(std::ostream& os) const {
    os << _offsetParam->id() <<  " ";
    Vector7d meas = internal::toVectorQT(_measurement);
    for (int i=0; i<7; i++) os  << meas[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3Prior::computeError() {
    Eigen::Isometry3d delta=_inverseMeasurement * _cache->n2w();
    _error = internal::toVectorMQT(delta);
  }

  void EdgeSE3Prior::linearizeOplus(){
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    Eigen::Isometry3d E;
    Eigen::Isometry3d Z, X, P;
    X=from->estimate();
    P=_cache->offsetParam()->offset();
    Z=_measurement;
    internal::computeEdgeSE3PriorGradient(E, _jacobianOplusXi, Z, X, P);
  }


  bool EdgeSE3Prior::setMeasurementFromState(){
    setMeasurement(_cache->n2w());
    return true;
  }


  void EdgeSE3Prior::initialEstimate(const OptimizableGraph::VertexSet& /*from_*/, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3 *v = static_cast<VertexSE3*>(_vertices[0]);
    assert(v && "Vertex for the Prior edge is not set");

    Eigen::Isometry3d newEstimate = _offsetParam->offset().inverse() * measurement();
    if (_information.block<3,3>(0,0).array().abs().sum() == 0){ // do not set translation, as that part of the information is all zero
      newEstimate.translation()=v->estimate().translation();
    }
    if (_information.block<3,3>(3,3).array().abs().sum() == 0){ // do not set rotation, as that part of the information is all zero
      newEstimate.matrix().block<3,3>(0,0) = internal::extractRotation(v->estimate());
    }
    v->setEstimate(newEstimate);
  }

}
