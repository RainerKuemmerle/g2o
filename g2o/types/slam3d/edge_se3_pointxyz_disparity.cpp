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

#include "edge_se3_pointxyz_disparity.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>
#include <string>
#include <typeinfo>

#include "g2o/core/parameter.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

// point to camera projection, monocular
EdgeSE3PointXYZDisparity::EdgeSE3PointXYZDisparity() {
  resizeParameters(1);
  installParameter<CacheCamera::ParameterType>(0);
  information().setIdentity();
  information()(2, 2) = 1000.;
}

bool EdgeSE3PointXYZDisparity::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  cache_ = resolveCache<CacheCamera>(vertexXn<0>(), "CACHE_CAMERA", pv);
  return cache_ != nullptr;
}

void EdgeSE3PointXYZDisparity::computeError() {
  // VertexSE3 *cam = static_cast<VertexSE3*>(vertices_[0]);
  VertexPointXYZ* point = vertexXnRaw<1>();
  const Vector3& pt = point->estimate();

  Vector3 p = cache_->w2i() * pt;

  Vector3 perr;
  perr.head<2>() = p.head<2>() / p(2);
  perr(2) = 1 / p(2);

  // error, which is backwards from the normal observed - calculated
  // measurement_ is the measured projection
  error_ = perr - measurement_;
}

#ifdef EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN

void EdgeSE3PointXYZDisparity::linearizeOplus() {
  // VertexSE3 *cam = static_cast<VertexSE3 *>(vertices_[0]);
  VertexPointXYZ* vp = vertexXnRaw<1>();

  const Vector3& pt = vp->estimate();

  Vector3 Zcam = cache_->w2l() * vp->estimate();

  using JacType = Eigen::Matrix<double, 3, 9>;
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

  // Eigen::Matrix<double,3,9,Eigen::ColMajor> Jprime =
  // vcache->params->Kcam_inverseOffsetR  * J;
  Jprime = cache_->camParams()->param().KcamInverseOffsetR() * Jprime;
  JacType Jhom;
  Vector3 Zprime = cache_->w2i() * pt;

  Jhom.block<2, 9>(0, 0) = 1 / (Zprime(2) * Zprime(2)) *
                           (Jprime.block<2, 9>(0, 0) * Zprime(2) -
                            Zprime.head<2>() * Jprime.block<1, 9>(2, 0));
  Jhom.block<1, 9>(2, 0) =
      -1 / (Zprime(2) * Zprime(2)) * Jprime.block<1, 9>(2, 0);

  jacobianOplusXi_ = Jhom.block<3, 6>(0, 0);
  jacobianOplusXj_ = Jhom.block<3, 3>(0, 6);
}

#endif

bool EdgeSE3PointXYZDisparity::setMeasurementFromState() {
  // VertexSE3 *cam = static_cast< VertexSE3*>(vertices_[0]);
  VertexPointXYZ* point = vertexXnRaw<1>();
  const Vector3& pt = point->estimate();

  Vector3 p = cache_->w2i() * pt;

  Vector3 perr;
  perr.head<2>() = p.head<2>() / p(2);
  perr(2) = 1 / p(2);

  // error, which is backwards from the normal observed - calculated
  // measurement_ is the measured projection
  measurement_ = perr;
  return true;
}

void EdgeSE3PointXYZDisparity::initialEstimate(
    const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/) {
  (void)from;
  assert(from.size() == 1 && from.count(vertices_[0]) == 1 &&
         "Can not initialize VertexDepthCam position by VertexTrackXYZ");
  VertexSE3* cam = vertexXnRaw<0>();
  VertexPointXYZ* point = vertexXnRaw<1>();

  const Matrix3& invKcam = cache_->camParams()->param().invKcam();
  Vector3 p;
  double w = 1. / measurement_(2);
  p.head<2>() = measurement_.head<2>() * w;
  p(2) = w;
  p = invKcam * p;
  p = cam->estimate() * (cache_->camParams()->param().offset() * p);
  point->setEstimate(p);
}

#ifdef G2O_HAVE_OPENGL
EdgeProjectDisparityDrawAction::EdgeProjectDisparityDrawAction()
    : DrawAction(typeid(EdgeSE3PointXYZDisparity).name()) {}

bool EdgeProjectDisparityDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;
  auto* e = static_cast<EdgeSE3PointXYZDisparity*>(&element);
  auto* fromEdge = static_cast<VertexSE3*>(e->vertices()[0].get());
  auto* toEdge = static_cast<VertexPointXYZ*>(e->vertices()[1].get());
  if (!fromEdge || !toEdge) return true;
  ParameterCamera* camParam =
      static_cast<ParameterCamera*>(e->parameter(0).get());
  Isometry3 fromTransform = fromEdge->estimate() * camParam->param().offset();
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
