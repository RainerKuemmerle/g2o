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

#include "parameter_se3_offset.h"

#include "isometry3d_gradients.h"
#include "vertex_se3.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

ParameterSE3Offset::ParameterSE3Offset() { setOffset(); }

void ParameterSE3Offset::setOffset(const Isometry3& offset) {
  offset_ = offset;
  inverseOffset_ = offset_.inverse();
}

bool ParameterSE3Offset::read(std::istream& is) {
  Vector7 off;
  bool state = internal::readVector(is, off);
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4::MapType(off.data() + 3).normalize();
  setOffset(internal::fromVectorQT(off));
  return state;
}

bool ParameterSE3Offset::write(std::ostream& os) const {
  return internal::writeVector(os, internal::toVectorQT(offset_));
}

void CacheSE3Offset::updateImpl() {
#ifndef NDEBUG
  auto* offsetParam = dynamic_cast<ParameterSE3Offset*>(parameters_[0].get());
#else
  auto* offsetParam = static_cast<ParameterSE3Offset*>(parameters_[0].get());
#endif

  const auto& v = static_cast<const VertexSE3&>(vertex());
  n2w_ = v.estimate() * offsetParam->offset();
  w2n_ = n2w_.inverse();
  w2l_ = v.estimate().inverse();
}

#ifdef G2O_HAVE_OPENGL
CacheSE3OffsetDrawAction::CacheSE3OffsetDrawAction()
    : DrawAction(typeid(CacheSE3Offset).name()) {}

bool CacheSE3OffsetDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    cubeSide_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::CUBE_SIDE", .05F);
  } else {
    cubeSide_ = nullptr;
  }
  return true;
}

bool CacheSE3OffsetDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  auto* that = static_cast<CacheSE3Offset*>(&element);
  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;
  float cs = cubeSide_ ? cubeSide_->value() : 1.0F;
  glPushAttrib(GL_COLOR);
  glColor3f(POSE_PARAMETER_COLOR);
  glPushMatrix();
  glMultMatrixd(that->offsetParam()->offset().cast<double>().data());
  opengl::drawBox(cs, cs, cs);
  glPopMatrix();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
