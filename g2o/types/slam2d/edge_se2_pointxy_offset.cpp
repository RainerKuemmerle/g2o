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

#include "parameter_se2_offset.h"
#include "edge_se2_pointxy_offset.h"
#include <iostream>

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeSE2PointXYOffset::EdgeSE2PointXYOffset() : BaseBinaryEdge<2, Vector2d, VertexSE2, VertexPointXY>() {
    information().setIdentity();
    cache = 0;
    offsetParam = 0;
    resizeParameters(1);
    installParameter(offsetParam, 0);
  }

  bool EdgeSE2PointXYOffset::resolveCaches(){
    ParameterVector pv(1);
    pv[0]=offsetParam;
    resolveCache(cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE2_OFFSET",pv);
    return cache != 0;
  }


  bool EdgeSE2PointXYOffset::read(std::istream& is) {
    int pId;
    is >> pId;
    setParameterId(0, pId);
    // measured keypoint
    Vector2d meas;
    for (int i=0; i<2; i++) is >> meas[i];
    setMeasurement(meas);
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

  bool EdgeSE2PointXYOffset::write(std::ostream& os) const {
    cerr <<"W";
    os << offsetParam->id() << " ";
    for (int i=0; i<2; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE2PointXYOffset::computeError() {
    // from cam to point (track)
    // VertexSE2 *rob = static_cast<VertexSE2*>(_vertices[0]);
    VertexPointXY *point = static_cast<VertexPointXY*>(_vertices[1]);

    Eigen::Vector2d perr = cache->w2lMatrix() * point->estimate();

    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _error = perr - _measurement;
    //    std::cout << _error << std::endl << std::endl;
  }

  void EdgeSE2PointXYOffset::linearizeOplus() {
    VertexSE2 *rob = static_cast<VertexSE2*>(_vertices[0]);
    VertexPointXY *point = static_cast<VertexPointXY*>(_vertices[1]);
    _jacobianOplusXi.block<2,2>(0,0) = - cache->RpInverseRInverseMatrix();
    _jacobianOplusXi.block<2,1>(0,2) = cache->RpInverseRInversePrimeMatrix()*(point->estimate()-rob->estimate().translation()); 
    _jacobianOplusXj = cache->RpInverseRInverseMatrix();
  }


  bool EdgeSE2PointXYOffset::setMeasurementFromState(){
    VertexPointXY *point = static_cast<VertexPointXY*>(_vertices[1]);

    const Vector2d &pt = point->estimate();

    Eigen::Vector2d perr = cache->w2lMatrix() * pt;
    _measurement = perr;
    return true;
  }


  void EdgeSE2PointXYOffset::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to_*/)
  {
    (void) from;
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexDepthCam position by VertexTrackXY");

    VertexSE2 *cam = dynamic_cast<VertexSE2*>(_vertices[0]);
    VertexPointXY *point = dynamic_cast<VertexPointXY*>(_vertices[1]);
    // SE2OffsetCache* vcache = (SE2OffsetCache* ) cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }
    // SE2OffsetParameters* params=vcache->params;
    Eigen::Vector2d p=_measurement;
    point->setEstimate(cam->estimate() * (offsetParam->offsetMatrix() * p));
  }

}
