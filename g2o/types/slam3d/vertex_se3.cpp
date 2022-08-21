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

#include "vertex_se3.h"

#include "g2o/core/factory.h"
#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <iostream>

#include "g2o/core/cache.h"

namespace g2o {

VertexSE3::VertexSE3() {
  setToOriginImpl();
  updateCache();
}

bool VertexSE3::read(std::istream& is) {
  Vector7 est;
  bool state = internal::readVector(is, est);
  setEstimate(internal::fromVectorQT(est));
  return state;
}

bool VertexSE3::write(std::ostream& os) const {
  return internal::writeVector(os, internal::toVectorQT(estimate()));
}

VertexSE3WriteGnuplotAction::VertexSE3WriteGnuplotAction()
    : WriteGnuplotAction(typeid(VertexSE3).name()) {}

bool VertexSE3WriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  auto* params = static_cast<WriteGnuplotAction::Parameters*>(params_.get());
  if (!params->os) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified"
              << std::endl;
    return false;
  }

  auto* v = static_cast<VertexSE3*>(&element);
  Vector6 est = internal::toVectorMQT(v->estimate());
  for (int i = 0; i < 6; i++) *(params->os) << est[i] << " ";
  *(params->os) << std::endl;
  return true;
}

#ifdef G2O_HAVE_OPENGL
void drawTriangle(float xSize, float ySize) {
  Vector3F p[3];
  glBegin(GL_TRIANGLES);
  p[0] << 0., 0., 0.;
  p[1] << -xSize, ySize, 0.;
  p[2] << -xSize, -ySize, 0.;
  for (int i = 1; i < 2; ++i) {
    Vector3F normal = (p[i] - p[0]).cross(p[i + 1] - p[0]);
    glNormal3f(normal.x(), normal.y(), normal.z());
    glVertex3f(p[0].x(), p[0].y(), p[0].z());
    glVertex3f(p[i].x(), p[i].y(), p[i].z());
    glVertex3f(p[i + 1].x(), p[i + 1].y(), p[i + 1].z());
  }
  glEnd();
}

VertexSE3DrawAction::VertexSE3DrawAction()
    : DrawAction(typeid(VertexSE3).name()),
      triangleX_(nullptr),
      triangleY_(nullptr) {
  cacheDrawActions_ = nullptr;
}

bool VertexSE3DrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    triangleX_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::TRIANGLE_X", .2F);
    triangleY_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::TRIANGLE_Y", .05F);
  } else {
    triangleX_ = nullptr;
    triangleY_ = nullptr;
  }
  return true;
}

bool VertexSE3DrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  initializeDrawActionsCache();
  refreshPropertyPtrs(params_);

  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* that = static_cast<VertexSE3*>(&element);

  glColor3f(POSE_VERTEX_COLOR);
  glPushMatrix();
  glMultMatrixd(that->estimate().matrix().cast<double>().eval().data());
  opengl::drawArrow2D(triangleX_->value(), triangleY_->value(),
                      triangleX_->value() * .3F);
  drawCache(that->cacheContainer(), params_);
  drawUserData(that->userData(), params_);
  glPopMatrix();
  return true;
}
#endif

}  // namespace g2o
