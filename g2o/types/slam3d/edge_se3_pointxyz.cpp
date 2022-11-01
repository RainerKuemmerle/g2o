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
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <iostream>

namespace g2o {

// point to camera projection, monocular
EdgeSE3PointXYZ::EdgeSE3PointXYZ() {
  information().setIdentity();
  resizeParameters(1);
  installParameter<CacheSE3Offset::ParameterType>(0);
}

bool EdgeSE3PointXYZ::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  cache_ = resolveCache<CacheSE3Offset>(vertexXn<0>(), "CACHE_SE3_OFFSET", pv);
  return cache_ != nullptr;
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
  // VertexSE3 *cam = static_cast<VertexSE3*>(vertices_[0]);
  VertexPointXYZ* point = vertexXnRaw<1>();

  const Vector3 perr = cache_->w2n() * point->estimate();

  // error, which is backwards from the normal observed - calculated
  // measurement_ is the measured projection
  error_ = perr - measurement_;
  //    std::cout << error_ << std::endl << std::endl;
}

void EdgeSE3PointXYZ::linearizeOplus() {
  // VertexSE3 *cam = static_cast<VertexSE3 *>(vertices_[0]);
  VertexPointXYZ* vp = vertexXnRaw<1>();

  Vector3 Zcam = cache_->w2l() * vp->estimate();

  using JacType = Eigen::Matrix<number_t, 3, 9>;

  JacType Jprime = JacType::Zero();
  Jprime.block<3, 3>(0, 0) = -Matrix3::Identity();

  //  J(0,3) = -0.0;
  Jprime(0, 4) = -2 * Zcam(2);
  Jprime(0, 5) = 2 * Zcam(1);

  Jprime(1, 3) = 2 * Zcam(2);
  //  J(1,4) = -0.0;
  Jprime(1, 5) = -2 * Zcam(0);

  Jprime(2, 3) = -2 * Zcam(1);
  Jprime(2, 4) = 2 * Zcam(0);
  //  J(2,5) = -0.0;

  Jprime.block<3, 3>(0, 6) = cache_->w2l().rotation();

  Eigen::Matrix<number_t, 3, 9, Eigen::ColMajor> Jhom =
      cache_->offsetParam()->inverseOffset().rotation() * Jprime;

  jacobianOplusXi_ = Jhom.block<3, 6>(0, 0);
  jacobianOplusXj_ = Jhom.block<3, 3>(0, 6);

  // std::cerr << "just linearized." << std::endl;
  // std::cerr << "jacobianOplusXi_:" << std::endl << jacobianOplusXi_ <<
  // std::endl; std::cerr << "jacobianOplusXj_:" << std::endl <<
  // jacobianOplusXj_ << std::endl;
}

bool EdgeSE3PointXYZ::setMeasurementFromState() {
  // VertexSE3 *cam = static_cast<VertexSE3*>(vertices_[0]);
  VertexPointXYZ* point = vertexXnRaw<1>();

  // calculate the projection
  const Vector3& pt = point->estimate();
  // SE3OffsetCache* vcache = (SE3OffsetCache*) cam->getCache(_cacheIds[0]);
  // if (! vcache){
  //   cerr << "fatal error in retrieving cache" << endl;
  // }

  const Vector3 perr = cache_->w2n() * pt;
  measurement_ = perr;
  return true;
}

void EdgeSE3PointXYZ::initialEstimate(const OptimizableGraph::VertexSet& from,
                                      OptimizableGraph::Vertex* to) {
  (void)from;
  (void)to;
  assert(from.size() == 1 && from.count(vertices_[0]) == 1 &&
         "Can not initialize VertexDepthCam position by VertexTrackXYZ");

  VertexSE3* cam = vertexXnRaw<0>();
  VertexPointXYZ* point = vertexXnRaw<1>();
  // SE3OffsetCache* vcache = (SE3OffsetCache* ) cam->getCache(_cacheIds[0]);
  // if (! vcache){
  //   cerr << "fatal error in retrieving cache" << endl;
  // }
  // SE3OffsetParameters* params=vcache->params;
  const Vector3 p = measurement_;
  point->setEstimate(cam->estimate() * (cache_->offsetParam()->offset() * p));
}

#ifdef G2O_HAVE_OPENGL
EdgeSE3PointXYZDrawAction::EdgeSE3PointXYZDrawAction()
    : DrawAction(typeid(EdgeSE3PointXYZ).name()) {}

bool EdgeSE3PointXYZDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* e = static_cast<EdgeSE3PointXYZ*>(&element);
  VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertex(0).get());
  VertexPointXYZ* toEdge = static_cast<VertexPointXYZ*>(e->vertex(1).get());
  if (!fromEdge || !toEdge) return true;
  ParameterSE3Offset* offsetParam =
      static_cast<ParameterSE3Offset*>(e->parameter(0).get());
  Isometry3 fromTransform = fromEdge->estimate() * offsetParam->offset();
  glColor3f(LANDMARK_EDGE_COLOR);
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glVertex3f(static_cast<float>(fromTransform.translation().x()),
             static_cast<float>(fromTransform.translation().y()),
             static_cast<float>(fromTransform.translation().z()));
  glVertex3f(static_cast<float>(toEdge->estimate().x()),
             static_cast<float>(toEdge->estimate().y()),
             static_cast<float>(toEdge->estimate().z()));
  glEnd();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
