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

#include "edge_se2.h"

#include <cmath>
#include <string>

#include "g2o/core/eigen_types.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

void EdgeSE2::initialEstimate(const OptimizableGraph::VertexSet& from,
                              OptimizableGraph::Vertex* /* to */) {
  auto fromEdge = vertexXn<0>();
  auto toEdge = vertexXn<1>();
  if (from.count(fromEdge) > 0)
    toEdge->setEstimate(fromEdge->estimate() * measurement_);
  else
    fromEdge->setEstimate(toEdge->estimate() * inverseMeasurement_);
}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgeSE2::linearizeOplus() {
  const VertexSE2* vi = vertexXnRaw<0>();
  const VertexSE2* vj = vertexXnRaw<1>();
  double thetai = vi->estimate().rotation().angle();

  Vector2 dt = vj->estimate().translation() - vi->estimate().translation();
  double si = std::sin(thetai);
  double ci = std::cos(thetai);

  jacobianOplusXi_ << -ci, -si, -si * dt.x() + ci * dt.y(), si, -ci,
      -ci * dt.x() - si * dt.y(), 0, 0, -1;

  jacobianOplusXj_ << ci, si, 0, -si, ci, 0, 0, 0, 1;

  const SE2& rmean = inverseMeasurement_;
  Matrix3 z;
  z.block<2, 2>(0, 0) = rmean.rotation().toRotationMatrix();
  z.col(2) << cst(0.), cst(0.), cst(1.);
  z.row(2).head<2>() << cst(0.), cst(0.);
  jacobianOplusXi_ = z * jacobianOplusXi_;
  jacobianOplusXj_ = z * jacobianOplusXj_;
}
#endif

#ifdef G2O_HAVE_OPENGL
EdgeSE2DrawAction::EdgeSE2DrawAction()
    : DrawAction(typeid(EdgeSE2).name()),
      triangleX_(nullptr),
      triangleY_(nullptr) {}

DrawAction::Parameters* EdgeSE2DrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters& params_) {
  DrawAction::Parameters* params = DrawAction::refreshPropertyPtrs(params_);
  if (params) {
    triangleX_ = params->makeProperty<FloatProperty>(
        typeName_ + "::GHOST_TRIANGLE_X", .2F);
    triangleY_ = params->makeProperty<FloatProperty>(
        typeName_ + "::GHOST_TRIANGLE_Y", .05F);
  }
  return params;
}

bool EdgeSE2DrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    HyperGraphElementAction::Parameters& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* e = static_cast<EdgeSE2*>(&element);
  auto from = e->vertexXn<0>();
  auto to = e->vertexXn<1>();
  if (!from && !to) return true;
  SE2 fromTransform;
  SE2 toTransform;
  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
  glDisable(GL_LIGHTING);
  if (!from) {
    glColor3f(POSE_EDGE_GHOST_COLOR);
    toTransform = to->estimate();
    fromTransform = to->estimate() * e->measurement().inverse();
    // DRAW THE FROM EDGE AS AN ARROW
    glPushMatrix();
    glTranslatef(fromTransform.translation().x(),
                 fromTransform.translation().y(), 0.F);
    glRotatef(RAD2DEG(fromTransform.rotation().angle()), 0.F, 0.F, 1.F);
    opengl::drawArrow2D(triangleX_->value(), triangleY_->value(),
                        triangleX_->value() * .3F);
    glPopMatrix();
  } else if (!to) {
    glColor3f(POSE_EDGE_GHOST_COLOR);
    fromTransform = from->estimate();
    toTransform = from->estimate() * e->measurement();
    // DRAW THE TO EDGE AS AN ARROW
    glPushMatrix();
    glTranslatef(toTransform.translation().x(), toTransform.translation().y(),
                 0.F);
    glRotatef(RAD2DEG(toTransform.rotation().angle()), 0.F, 0.F, 1.F);
    opengl::drawArrow2D(triangleX_->value(), triangleY_->value(),
                        triangleX_->value() * .3F);
    glPopMatrix();
  } else {
    glColor3f(POSE_EDGE_COLOR);
    fromTransform = from->estimate();
    toTransform = to->estimate();
  }
  glBegin(GL_LINES);
  glVertex3f(fromTransform.translation().x(), fromTransform.translation().y(),
             0.F);
  glVertex3f(toTransform.translation().x(), toTransform.translation().y(), 0.F);
  glEnd();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
