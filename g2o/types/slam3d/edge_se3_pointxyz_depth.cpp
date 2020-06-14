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

#include "edge_se3_pointxyz_depth.h"

namespace g2o {

  // point to camera projection, monocular
  EdgeSE3PointXYZDepth::EdgeSE3PointXYZDepth()
      : BaseBinaryEdge<3, Vector3, VertexSE3, VertexPointXYZ>(), cache(nullptr) {
    resizeParameters(1);
    installParameter(params, 0);
    information().setIdentity();
    information()(2,2)=100;
    J.fill(0);
    J.block<3,3>(0,0) = -Matrix3::Identity();
  }

  bool EdgeSE3PointXYZDepth::resolveCaches(){
    ParameterVector pv(1);
    pv[0]=params;
    resolveCache(cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_CAMERA",pv);
    return cache != 0;
  }

  bool EdgeSE3PointXYZDepth::read(std::istream& is) {
    readParamIds(is);
    internal::readVector(is, _measurement); // measured keypoint
    return readInformationMatrix(is);
  }

  bool EdgeSE3PointXYZDepth::write(std::ostream& os) const {
    writeParamIds(os);
    internal::writeVector(os, measurement());
    return writeInformationMatrix(os);
  }

  void EdgeSE3PointXYZDepth::computeError() {
    // from cam to point (track)
    //VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = static_cast<VertexPointXYZ*>(_vertices[1]);

    Vector3 p = cache->w2i() * point->estimate();
    Vector3 perr;
    perr.head<2>() = p.head<2>()/p(2);
    perr(2) = p(2);

    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _error = perr - _measurement;
  }

  void EdgeSE3PointXYZDepth::linearizeOplus() {
    //VertexSE3 *cam = static_cast<VertexSE3 *>(_vertices[0]);
    VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(_vertices[1]);

    const Vector3& pt = vp->estimate();

    Vector3 Zcam = cache->w2l() * pt;

    //  J(0,3) = -0.0;
    J(0,4) = -2*Zcam(2);
    J(0,5) = 2*Zcam(1);

    J(1,3) = 2*Zcam(2);
    //  J(1,4) = -0.0;
    J(1,5) = -2*Zcam(0);

    J(2,3) = -2*Zcam(1);
    J(2,4) = 2*Zcam(0);
    //  J(2,5) = -0.0;

    J.block<3,3>(0,6) = cache->w2l().rotation();

    Eigen::Matrix<number_t,3,9,Eigen::ColMajor> Jprime = params->Kcam_inverseOffsetR()  * J;
    Vector3 Zprime = cache->w2i() * pt;

    Eigen::Matrix<number_t,3,9,Eigen::ColMajor> Jhom;
    Jhom.block<2,9>(0,0) = 1/(Zprime(2)*Zprime(2)) * (Jprime.block<2,9>(0,0)*Zprime(2) - Zprime.head<2>() * Jprime.block<1,9>(2,0));
    Jhom.block<1,9>(2,0) = Jprime.block<1,9>(2,0);

    _jacobianOplusXi = Jhom.block<3,6>(0,0);
    _jacobianOplusXj = Jhom.block<3,3>(0,6);
  }


  bool EdgeSE3PointXYZDepth::setMeasurementFromState(){
    //VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = static_cast<VertexPointXYZ*>(_vertices[1]);

    // calculate the projection
    const Vector3& pt = point->estimate();

    Vector3 p = cache->w2i() * pt;
    Vector3 perr;
    perr.head<2>() = p.head<2>()/p(2);
    perr(2) = p(2);
    _measurement = perr;
    return true;
  }


  void EdgeSE3PointXYZDepth::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to_*/)
  {
    (void) from;
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexDepthCam position by VertexTrackXYZ");

    VertexSE3 *cam = dynamic_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = dynamic_cast<VertexPointXYZ*>(_vertices[1]);
    const Eigen::Matrix<number_t, 3, 3, Eigen::ColMajor>& invKcam = params->invKcam();
    Vector3 p;
    p(2) = _measurement(2);
    p.head<2>() = _measurement.head<2>()*p(2);
    p=invKcam*p;
    point->setEstimate(cam->estimate() * (params->offset() * p));
  }

}
