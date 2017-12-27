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
#include "g2o/types/slam3d/isometry3d_gradients.h"
#include <iostream>

namespace g2o {
namespace deprecated {

  using namespace std;


  // point to camera projection, monocular
  EdgeSE3Prior::EdgeSE3Prior() : BaseUnaryEdge<6, SE3Quat, VertexSE3>() {
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
    Vector7 meas;
    for (int i=0; i<7; i++) is >> meas[i];
    setMeasurement(SE3Quat(meas));
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
    for (int i=0; i<7; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3Prior::computeError() {
    SE3Quat delta=_inverseMeasurement * _cache->n2w();

    _error.head<3>() = delta.translation();
    // The analytic Jacobians assume the error in this special form (w beeing positive)
    if (delta.rotation().w() < 0.)
      _error.tail<3>() =  - delta.rotation().vec();
    else
      _error.tail<3>() =  delta.rotation().vec();
  }

  void EdgeSE3Prior::linearizeOplus(){
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    Isometry3 E;
    Isometry3 Z, X, P;
    X=from->estimate().rotation().toRotationMatrix();
    X.translation()=from->estimate().translation();
    P=_cache->offsetParam()->offset().rotation().toRotationMatrix();
    P.translation()=_cache->offsetParam()->offset().translation();
    Z=_measurement.rotation().toRotationMatrix();
    Z.translation()=_measurement.translation();
    internal::computeEdgeSE3PriorGradient(E, _jacobianOplusXi, Z, X, P);
  }


  bool EdgeSE3Prior::setMeasurementFromState(){
    setMeasurement(_cache->n2w());
    return true;
  }


  void EdgeSE3Prior::initialEstimate(const OptimizableGraph::VertexSet& /*from_*/, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3 *v = static_cast<VertexSE3*>(_vertices[0]);

    SE3Quat newEstimate = _offsetParam->offset().inverse() * measurement();
    if (_information.block<3,3>(0,0).squaredNorm()==0){ // do not set translation
      newEstimate.setTranslation(v->estimate().translation());
    }
    if (_information.block<3,3>(3,3).squaredNorm()==0){ // do not set rotation
      newEstimate.setRotation(v->estimate().rotation());
    }
    v->setEstimate(newEstimate);
  }

}
}
