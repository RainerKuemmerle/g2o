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

#include "edge_se3_pointxyz.h"
#include "parameter_se3_offset.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <iostream>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o {
  using namespace std;

  // point to camera projection, monocular
  EdgeSE3PointXYZ::EdgeSE3PointXYZ() : BaseBinaryEdge<3, Vector3, VertexSE3, VertexPointXYZ>() {
    information().setIdentity();
    J.fill(0);
    J.block<3,3>(0,0) = -Matrix3::Identity();
    cache = 0;
    offsetParam = 0;
    resizeParameters(1);
    installParameter(offsetParam, 0);
  }

  bool EdgeSE3PointXYZ::resolveCaches(){
    ParameterVector pv(1);
    pv[0]=offsetParam;
    resolveCache(cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE3_OFFSET",pv);
    return cache != 0;
  }

  bool EdgeSE3PointXYZ::read(std::istream& is) {
    readParamIds(is);
    Vector3 meas;
    internal::readVector(is, meas);
    setMeasurement(meas);
    readInformationMatrix(is);
    return is.good() || is.eof();
  }

  bool EdgeSE3PointXYZ::write(std::ostream& os) const {
    bool state = writeParamIds(os);
    state &= internal::writeVector(os, measurement());
    state &= writeInformationMatrix(os);
    return state;
  }

  void EdgeSE3PointXYZ::computeError() {
    // from cam to point (track)
    //VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = static_cast<VertexPointXYZ*>(_vertices[1]);

    Vector3 perr = cache->w2n() * point->estimate();

    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _error = perr - _measurement;
    //    std::cout << _error << std::endl << std::endl;
  }

  void EdgeSE3PointXYZ::linearizeOplus() {
    //VertexSE3 *cam = static_cast<VertexSE3 *>(_vertices[0]);
    VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(_vertices[1]);

    Vector3 Zcam = cache->w2l() * vp->estimate();

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

    Eigen::Matrix<number_t,3,9,Eigen::ColMajor> Jhom = offsetParam->inverseOffset().rotation() * J;

    _jacobianOplusXi = Jhom.block<3,6>(0,0);
    _jacobianOplusXj = Jhom.block<3,3>(0,6);

    // std::cerr << "just linearized." << std::endl;
    // std::cerr << "_jacobianOplusXi:" << std::endl << _jacobianOplusXi << std::endl;
    // std::cerr << "_jacobianOplusXj:" << std::endl << _jacobianOplusXj << std::endl;
  }


  bool EdgeSE3PointXYZ::setMeasurementFromState(){
    //VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = static_cast<VertexPointXYZ*>(_vertices[1]);

    // calculate the projection
    const Vector3 &pt = point->estimate();
    // SE3OffsetCache* vcache = (SE3OffsetCache*) cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }

    Vector3 perr = cache->w2n() * pt;
    _measurement = perr;
    return true;
  }


  void EdgeSE3PointXYZ::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
  {
    (void) from; (void) to;
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexDepthCam position by VertexTrackXYZ");

    VertexSE3 *cam = dynamic_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = dynamic_cast<VertexPointXYZ*>(_vertices[1]);
    // SE3OffsetCache* vcache = (SE3OffsetCache* ) cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }
    // SE3OffsetParameters* params=vcache->params;
    Vector3 p=_measurement;
    point->setEstimate(cam->estimate() * (offsetParam->offset() * p));
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE3PointXYZDrawAction::EdgeSE3PointXYZDrawAction(): DrawAction(typeid(EdgeSE3PointXYZ).name()){}

  HyperGraphElementAction* EdgeSE3PointXYZDrawAction::operator()(HyperGraph::HyperGraphElement* element,
               HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;

    EdgeSE3PointXYZ* e =  static_cast<EdgeSE3PointXYZ*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertex(0));
    VertexPointXYZ* toEdge   = static_cast<VertexPointXYZ*>(e->vertex(1));
    if (! fromEdge || ! toEdge)
      return this;
    Isometry3 fromTransform=fromEdge->estimate() * e->offsetParameter()->offset();
    glColor3f(LANDMARK_EDGE_COLOR);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)fromTransform.translation().x(),(float)fromTransform.translation().y(),(float)fromTransform.translation().z());
    glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),(float)toEdge->estimate().z());
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

} // end namespace
