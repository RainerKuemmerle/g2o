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

#include "isometry3d_gradients.h"
#include "isometry3d_mappings.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

ParameterCamera::ParameterCamera() {
  setId(-1);
  setKcam(1, 1, 0.5, 0.5);
  setOffset();
}

void ParameterCamera::setOffset(const Isometry3& offset_) {
  ParameterSE3Offset::setOffset(offset_);
  Kcam_inverseOffsetR_ = Kcam_ * inverseOffset().rotation();
}

void ParameterCamera::setKcam(number_t fx, number_t fy, number_t cx,
                              number_t cy) {
  Kcam_.setZero();
  Kcam_(0, 0) = fx;
  Kcam_(1, 1) = fy;
  Kcam_(0, 2) = cx;
  Kcam_(1, 2) = cy;
  Kcam_(2, 2) = 1.0;
  invKcam_ = Kcam_.inverse();
  Kcam_inverseOffsetR_ = Kcam_ * inverseOffset().rotation();
}

bool ParameterCamera::read(std::istream& is) {
  Vector7 off;
  internal::readVector(is, off);
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4::MapType(off.data() + 3).normalize();
  setOffset(internal::fromVectorQT(off));
  number_t fx;
  number_t fy;
  number_t cx;
  number_t cy;
  is >> fx >> fy >> cx >> cy;
  setKcam(fx, fy, cx, cy);
  return is.good();
}

bool ParameterCamera::write(std::ostream& os) const {
  internal::writeVector(os, internal::toVectorQT(offset_));
  os << Kcam_(0, 0) << " ";
  os << Kcam_(1, 1) << " ";
  os << Kcam_(0, 2) << " ";
  os << Kcam_(1, 2) << " ";
  return os.good();
}

void CacheCamera::updateImpl() {
  CacheSE3Offset::updateImpl();
  w2i_.matrix().topLeftCorner<3, 4>() =
      camParams()->Kcam() * w2n().matrix().topLeftCorner<3, 4>();
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

  glPushAttrib(GL_COLOR);
  glColor3f(POSE_PARAMETER_COLOR);
  glPushMatrix();
  glMultMatrixd(that->camParams()->offset().cast<double>().data());
  glRotatef(180.0F, 0.0F, 1.0F, 0.0F);
  opengl::drawPyramid(cameraSide_->value(), cameraZ_->value());
  glPopMatrix();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
