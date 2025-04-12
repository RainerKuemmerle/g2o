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
  bool state = internal::writeVector(os, _estimate.toVector());
  state &= internal::writeVector(os, color);
  return state;
}

#ifdef G2O_HAVE_OPENGL

VertexPlaneDrawAction::VertexPlaneDrawAction()
    : DrawAction(typeid(VertexPlane).name()),
      _planeWidth(nullptr),
      _planeHeight(nullptr) {}

bool VertexPlaneDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams) {
    _planeWidth = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::PLANE_WIDTH", 3);
    _planeHeight = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::PLANE_HEIGHT", 3);
  } else {
    _planeWidth = 0;
    _planeHeight = 0;
  }
  return true;
}

HyperGraphElementAction* VertexPlaneDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  refreshPropertyPtrs(params_);
  if (!_previousParams) return this;
  if (_show && !_show->value()) return this;

  if (_planeWidth && _planeHeight) {
    VertexPlane* that = static_cast<VertexPlane*>(element);
    double d = that->estimate().distance();
    double azimuth = Plane3D::azimuth(that->estimate().normal());
    double elevation = Plane3D::elevation(that->estimate().normal());
    glColor3f(float(that->color(0)), float(that->color(1)),
              float(that->color(2)));
    glPushMatrix();
    glRotatef(float(RAD2DEG(azimuth)), 0.f, 0.f, 1.f);
    glRotatef(float(RAD2DEG(elevation)), 0.f, -1.f, 0.f);
    glTranslatef(float(d), 0.f, 0.f);

    glBegin(GL_QUADS);
    glNormal3f(-1.f, 0.f, 0.f);
    glVertex3f(0.f, -_planeWidth->value(), -_planeHeight->value());
    glVertex3f(0.f, _planeWidth->value(), -_planeHeight->value());
    glVertex3f(0.f, _planeWidth->value(), _planeHeight->value());
    glVertex3f(0.f, -_planeWidth->value(), _planeHeight->value());
    glEnd();
    glPopMatrix();
  }

  return this;
}
#endif

}  // namespace g2o
