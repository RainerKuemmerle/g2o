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

#include "vertex_line3d.h"

#include "g2o/stuff/misc.h"
#include "g2o/stuff/opengl_wrapper.h"

namespace g2o {

VertexLine3D::VertexLine3D() : color(1., 0.5, 0.) {}

bool VertexLine3D::read(std::istream& is) {
  Vector6 lv;
  bool state = internal::readVector(is, lv);
  setEstimate(Line3D(lv));
  return state;
}

bool VertexLine3D::write(std::ostream& os) const {
  return internal::writeVector(os, estimate_);
}

#ifdef G2O_HAVE_OPENGL
VertexLine3DDrawAction::VertexLine3DDrawAction()
    : DrawAction(typeid(VertexLine3D).name()),
      lineLength_(nullptr),
      lineWidth_(nullptr) {}

bool VertexLine3DDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) {
    return false;
  }
  if (previousParams_) {
    lineLength_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::LINE_LENGTH", 15);
    lineWidth_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::LINE_WIDTH", 5);
  } else {
    lineLength_ = nullptr;
    lineWidth_ = nullptr;
  }
  return true;
}

bool VertexLine3DDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) {
    return false;
  }

  refreshPropertyPtrs(params_);
  if (!previousParams_) {
    return true;
  }

  if (show_ && !show_->value()) {
    return true;
  }

  auto* that = static_cast<VertexLine3D*>(&element);
  Line3D line = that->estimate();
  line.normalize();
  Vector3 direction = line.d();
  Vector3 npoint = line.d().cross(line.w());
  glPushMatrix();
  glColor3f(static_cast<float>(that->color(0)),
            static_cast<float>(that->color(1)),
            static_cast<float>(that->color(2)));
  if (lineLength_ && lineWidth_) {
    glLineWidth(static_cast<float>(lineWidth_->value()));
    glBegin(GL_LINES);
    glNormal3f(static_cast<float>(npoint.x()), static_cast<float>(npoint.y()),
               static_cast<float>(npoint.z()));
    glVertex3f(static_cast<float>(npoint.x() -
                                  direction.x() * lineLength_->value() / 2),
               static_cast<float>(npoint.y() -
                                  direction.y() * lineLength_->value() / 2),
               static_cast<float>(npoint.z() -
                                  direction.z() * lineLength_->value() / 2));
    glVertex3f(static_cast<float>(npoint.x() +
                                  direction.x() * lineLength_->value() / 2),
               static_cast<float>(npoint.y() +
                                  direction.y() * lineLength_->value() / 2),
               static_cast<float>(npoint.z() +
                                  direction.z() * lineLength_->value() / 2));
    glEnd();
  }
  glPopMatrix();

  return true;
}
#endif

}  // namespace g2o
