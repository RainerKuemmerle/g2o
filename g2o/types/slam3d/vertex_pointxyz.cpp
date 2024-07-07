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

#include "vertex_pointxyz.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <string>

namespace g2o {

#ifdef G2O_HAVE_OPENGL
VertexPointXYZDrawAction::VertexPointXYZDrawAction()
    : DrawAction(typeid(VertexPointXYZ).name()), pointSize_(nullptr) {}

DrawAction::Parameters* VertexPointXYZDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters& params_) {
  DrawAction::Parameters* params = DrawAction::refreshPropertyPtrs(params_);
  if (params) {
    pointSize_ =
        params->makeProperty<FloatProperty>(typeName_ + "::POINT_SIZE", 1.);
  }
  return params;
}

bool VertexPointXYZDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    HyperGraphElementAction::Parameters& params) {
  if (typeid(element).name() != typeName_) return false;
  initializeDrawActionsCache();
  refreshPropertyPtrs(params);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;
  auto* that = static_cast<VertexPointXYZ*>(&element);

  glPushMatrix();
  glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
  glDisable(GL_LIGHTING);
  glColor3f(LANDMARK_VERTEX_COLOR);
  float ps = pointSize_ ? pointSize_->value() : 1.F;
  glTranslatef(static_cast<float>(that->estimate()(0)),
               static_cast<float>(that->estimate()(1)),
               static_cast<float>(that->estimate()(2)));
  opengl::drawPoint(ps);
  glPopAttrib();
  drawCache(that->cacheContainer(), params);
  drawUserData(that->userData(), params);
  glPopMatrix();
  return true;
}
#endif

}  // namespace g2o
