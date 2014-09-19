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
#include "isometry3d_gradients.h"
#include "parameter_se3_offset.h"

#include <iostream>

namespace g2o {
  using namespace std;
  using namespace Eigen;

  EdgeSE3Offset::EdgeSE3Offset() : EdgeSE3() {
    information().setIdentity();
    _offsetFrom = 0;
    _offsetTo = 0;
    _cacheFrom = 0;
    _cacheTo = 0;
    resizeParameters(2);
    installParameter(_offsetFrom, 0);
    installParameter(_offsetTo, 1);
  }

  bool EdgeSE3Offset::resolveCaches(){
    assert(_offsetFrom && _offsetTo);

    ParameterVector pv(2);
    pv[0]=_offsetFrom;
    resolveCache(_cacheFrom, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE3_OFFSET",pv);
    pv[1]=_offsetTo;
    resolveCache(_cacheTo, (OptimizableGraph::Vertex*)_vertices[1],"CACHE_SE3_OFFSET",pv);
    return (_cacheFrom && _cacheTo);
  }

  bool EdgeSE3Offset::read(std::istream& is) {
    int pidFrom, pidTo;
    is >> pidFrom >> pidTo   ;
    if (! setParameterId(0,pidFrom))
      return false;
    if (! setParameterId(1,pidTo))
      return false;

    Vector7d meas;
    for (int i=0; i<7; i++)
      is >> meas[i];
    // normalize the quaternion to recover numerical precision lost by storing as human readable text
    Vector4D::MapType(meas.data()+3).normalize();
    setMeasurement(internal::fromVectorQT(meas));

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
      //  we overwrite the information matrix with the Identity
      information().setIdentity();
    }
    return true;
  }

  bool EdgeSE3Offset::write(std::ostream& os) const {
    Vector7d meas=internal::toVectorQT(_measurement);
    for (int i=0; i<7; i++) os  << meas[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }

  void EdgeSE3Offset::computeError() {
    Isometry3D delta=_inverseMeasurement * _cacheFrom->w2n() * _cacheTo->n2w();
    _error=internal::toVectorMQT(delta);
  }

  bool EdgeSE3Offset::setMeasurementFromState(){
    Isometry3D delta = _cacheFrom->w2n() * _cacheTo->n2w();
    setMeasurement(delta);
    return true;
  }

  void EdgeSE3Offset::linearizeOplus(){
    //BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>::linearizeOplus();

    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *to   = static_cast<VertexSE3*>(_vertices[1]);
    Isometry3D E;
    const Isometry3D& Xi=from->estimate();
    const Isometry3D& Xj=to->estimate();
    const Isometry3D& Pi=_cacheFrom->offsetParam()->offset();
    const Isometry3D& Pj=_cacheTo->offsetParam()->offset();
    const Isometry3D& Z=_measurement;
    // Matrix6d Ji, Jj;
    // computeSE3Gradient(E, Ji , Jj,
    //                    Z, Pi, Xi, Pj, Xj);
    // cerr  << "Ji:" << endl;
    // cerr << Ji-_jacobianOplusXi << endl;
    // cerr  << "Jj:" << endl;
    // cerr << Jj-_jacobianOplusXj << endl;
    internal::computeEdgeSE3Gradient(E, _jacobianOplusXi , _jacobianOplusXj, Z, Xi, Xj, Pi, Pj);
  }

  void EdgeSE3Offset::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *to   = static_cast<VertexSE3*>(_vertices[1]);

    Isometry3D virtualMeasurement = _cacheFrom->offsetParam()->offset() * measurement() * _cacheTo->offsetParam()->offset().inverse();

    if (from_.count(from) > 0) {
      to->setEstimate(from->estimate() * virtualMeasurement);
    } else
      from->setEstimate(to->estimate() * virtualMeasurement.inverse());
  }

}
