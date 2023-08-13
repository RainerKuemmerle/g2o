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

#include "vertex_se2.h"

#include <typeinfo>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

void VertexSE2::oplusImpl(const VectorX::MapType& update) {
  Vector2 t = estimate_.translation();
  t += update.head<2>();
  double angle = normalize_theta(estimate_.rotation().angle() + update[2]);
  estimate_.setTranslation(t);
  estimate_.setRotation(Rotation2D(angle));
}

#ifdef G2O_HAVE_OPENGL
VertexSE2DrawAction::VertexSE2DrawAction()
    : DrawAction(typeid(VertexSE2).name()),
      triangleX_(nullptr),
      triangleY_(nullptr) {}

bool VertexSE2DrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& parameters) {
  if (!DrawAction::refreshPropertyPtrs(parameters)) return false;
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

bool VertexSE2DrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& parameters) {
  if (typeid(element).name() != typeName_) return false;
  initializeDrawActionsCache();
  refreshPropertyPtrs(parameters);

  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* that = static_cast<VertexSE2*>(&element);

  glColor3f(POSE_VERTEX_COLOR);
  glPushMatrix();
  glTranslatef(static_cast<float>(that->estimate().translation().x()),
               static_cast<float>(that->estimate().translation().y()), 0.F);
  glRotatef(static_cast<float> RAD2DEG(that->estimate().rotation().angle()),
            0.F, 0.F, 1.F);
  opengl::drawArrow2D(static_cast<float>(triangleX_->value()),
                      static_cast<float>(triangleY_->value()),
                      static_cast<float>(triangleX_->value()) * .3F);
  drawCache(that->cacheContainer(), parameters);
  drawUserData(that->userData(), parameters);
  glPopMatrix();
  return true;
}
#endif

}  // namespace g2o
