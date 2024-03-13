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

#include "parameter_camera.h"

#include <Eigen/src/Geometry/Transform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <string>
#include <typeinfo>

#include "g2o/types/slam3d/vertex_se3.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

void ParameterCamera::update() {}

void CacheCamera::updateImpl() {
#ifndef NDEBUG
  auto* offsetParam = dynamic_cast<ParameterCamera*>(parameters_[0].get());
#else
  auto* offsetParam = static_cast<ParameterCamera*>(parameters_[0].get());
#endif

  const auto& v = static_cast<const VertexSE3&>(vertex());
  n2w_ = v.estimate() * offsetParam->param().offset();
  w2n_ = n2w_.inverse();
  w2l_ = v.estimate().inverse();

  w2i_.matrix().topLeftCorner<3, 4>() =
      offsetParam->param().Kcam() * w2n().matrix().topLeftCorner<3, 4>();
}

#ifdef G2O_HAVE_OPENGL
CacheCameraDrawAction::CacheCameraDrawAction()
    : DrawAction(typeid(CacheCamera).name()) {}

bool CacheCameraDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    cameraZ_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::CAMERA_Z", .05F);
    cameraSide_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::CAMERA_SIDE", .05F);

  } else {
    cameraZ_ = nullptr;
    cameraSide_ = nullptr;
  }
  return true;
}

bool CacheCameraDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params) {
  if (typeid(element).name() != typeName_) return false;
  auto* that = static_cast<CacheCamera*>(&element);
  refreshPropertyPtrs(params);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* offsetParam =
      static_cast<ParameterCamera*>(that->parameters()[0].get());
  glPushAttrib(GL_COLOR);
  glColor3f(POSE_PARAMETER_COLOR);
  glPushMatrix();
  glMultMatrixd(offsetParam->param().offset().data());
  glRotatef(180.0F, 0.0F, 1.0F, 0.0F);
  opengl::drawPyramid(cameraSide_->value(), cameraZ_->value());
  glPopMatrix();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
