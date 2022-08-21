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

#include "vertex_point_xy.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <typeinfo>

#include "g2o/stuff/macros.h"

namespace g2o {

VertexPointXY::VertexPointXY()

{
  estimate_.setZero();
}

bool VertexPointXY::read(std::istream& is) {
  return internal::readVector(is, estimate_);
}

bool VertexPointXY::write(std::ostream& os) const {
  return internal::writeVector(os, estimate());
}

VertexPointXYWriteGnuplotAction::VertexPointXYWriteGnuplotAction()
    : WriteGnuplotAction(typeid(VertexPointXY).name()) {}

bool VertexPointXYWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params) {
  if (typeid(element).name() != typeName_) return false;

  auto* gnuplot_params = static_cast<WriteGnuplotAction::Parameters*>(params.get());
  if (!gnuplot_params->os) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified"
              << std::endl;
    return false;
  }

  auto* v = static_cast<VertexPointXY*>(&element);
  *(gnuplot_params->os) << v->estimate().x() << " " << v->estimate().y()
                        << std::endl;
  return true;
}

#ifdef G2O_HAVE_OPENGL
VertexPointXYDrawAction::VertexPointXYDrawAction()
    : DrawAction(typeid(VertexPointXY).name()), pointSize_(nullptr) {}

bool VertexPointXYDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params) {
  if (!DrawAction::refreshPropertyPtrs(params)) return false;
  if (previousParams_) {
    pointSize_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::POINT_SIZE", 1.);
  } else {
    pointSize_ = nullptr;
  }
  return true;
}

bool VertexPointXYDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params) {
  if (typeid(element).name() != typeName_) return false;
  initializeDrawActionsCache();
  refreshPropertyPtrs(params);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;
  auto* that = static_cast<VertexPointXY*>(&element);

  glPushMatrix();
  glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
  glDisable(GL_LIGHTING);
  glColor3f(LANDMARK_VERTEX_COLOR);
  float ps = pointSize_ ? pointSize_->value() : 1.0F;
  glTranslatef(that->estimate()(0), that->estimate()(1), 0.0F);
  opengl::drawPoint(ps);
  glPopAttrib();
  drawCache(that->cacheContainer(), params);
  drawUserData(that->userData(), params);
  glPopMatrix();
  return true;
}
#endif

}  // namespace g2o
