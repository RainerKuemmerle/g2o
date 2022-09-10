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

#include "vertex_plane.h"

#include "g2o/stuff/opengl_wrapper.h"

namespace g2o {

VertexPlane::VertexPlane() { color << cst(.2), cst(.2), cst(.2); }

bool VertexPlane::read(std::istream& is) {
  Vector4 lv;
  bool state = internal::readVector(is, lv);
  setEstimate(Plane3D(lv));
  state &= internal::readVector(is, color);
  return state;
}

bool VertexPlane::write(std::ostream& os) const {
  bool state = internal::writeVector(os, estimate_.toVector());
  state &= internal::writeVector(os, color);
  return state;
}

#ifdef G2O_HAVE_OPENGL

VertexPlaneDrawAction::VertexPlaneDrawAction()
    : DrawAction(typeid(VertexPlane).name()),
      planeWidth_(nullptr),
      planeHeight_(nullptr) {}

bool VertexPlaneDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    planeWidth_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::PLANE_WIDTH", 3);
    planeHeight_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::PLANE_HEIGHT", 3);
  } else {
    planeWidth_ = nullptr;
    planeHeight_ = nullptr;
  }
  return true;
}

bool VertexPlaneDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;
  if (show_ && !show_->value()) return true;

  if (planeWidth_ && planeHeight_) {
    auto* that = static_cast<VertexPlane*>(&element);
    number_t d = that->estimate().distance();
    number_t azimuth = Plane3D::azimuth(that->estimate().normal());
    number_t elevation = Plane3D::elevation(that->estimate().normal());
    glColor3f(static_cast<float>(that->color(0)),
              static_cast<float>(that->color(1)),
              static_cast<float>(that->color(2)));
    glPushMatrix();
    glRotatef(static_cast<float>(RAD2DEG(azimuth)), 0.F, 0.F, 1.F);
    glRotatef(static_cast<float>(RAD2DEG(elevation)), 0.F, -1.F, 0.F);
    glTranslatef(static_cast<float>(d), 0.F, 0.F);

    glBegin(GL_QUADS);
    glNormal3f(-1.F, 0.F, 0.F);
    glVertex3f(0.F, -planeWidth_->value(), -planeHeight_->value());
    glVertex3f(0.F, planeWidth_->value(), -planeHeight_->value());
    glVertex3f(0.F, planeWidth_->value(), planeHeight_->value());
    glVertex3f(0.F, -planeWidth_->value(), planeHeight_->value());
    glEnd();
    glPopMatrix();
  }

  return true;
}
#endif

}  // namespace g2o
