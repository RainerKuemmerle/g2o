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

#include "edge_se2_pointxy_bearing.h"

#include <cassert>

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

EdgeSE2PointXYBearing::EdgeSE2PointXYBearing() {}

void EdgeSE2PointXYBearing::initialEstimate(
    const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/) {
  assert(from.size() == 1 && from.count(_vertices[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexPointXY");

  if (from.count(_vertices[0]) != 1) return;
  double r = 2.;
  const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
  VertexPointXY* l2 = static_cast<VertexPointXY*>(_vertices[1]);
  SE2 t = v1->estimate();
  t.setRotation(t.rotation() * Rotation2D(_measurement));
  Vector2 vr(r, 0.);
  l2->setEstimate(t * vr);
}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgeSE2PointXYBearing::linearizeOplus() {
  const VertexSE2* vi = static_cast<const VertexSE2*>(_vertices[0]);
  const VertexPointXY* vj = static_cast<const VertexPointXY*>(_vertices[1]);
  const double& x1 = vi->estimate().translation()[0];
  const double& y1 = vi->estimate().translation()[1];
  const double& x2 = vj->estimate()[0];
  const double& y2 = vj->estimate()[1];

  double aux = (std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));

  _jacobianOplusXi(0, 0) = (y1 - y2) / aux;
  _jacobianOplusXi(0, 1) = (x2 - x1) / aux;
  _jacobianOplusXi(0, 2) = 1;

  _jacobianOplusXj(0, 0) = (y2 - y1) / aux;
  _jacobianOplusXj(0, 1) = (x1 - x2) / aux;
}
#endif

bool EdgeSE2PointXYBearing::read(std::istream& is) {
  is >> _measurement >> information()(0, 0);
  return true;
}

bool EdgeSE2PointXYBearing::write(std::ostream& os) const {
  os << measurement() << " " << information()(0, 0);
  return os.good();
}

EdgeSE2PointXYBearingWriteGnuplotAction::
    EdgeSE2PointXYBearingWriteGnuplotAction()
    : WriteGnuplotAction(typeid(EdgeSE2PointXYBearing).name()) {}

HyperGraphElementAction* EdgeSE2PointXYBearingWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  WriteGnuplotAction::Parameters* params =
      static_cast<WriteGnuplotAction::Parameters*>(params_);
  if (!params->os) {
    return nullptr;
  }

  EdgeSE2PointXYBearing* e = static_cast<EdgeSE2PointXYBearing*>(element);
  VertexSE2* fromEdge = static_cast<VertexSE2*>(e->vertex(0));
  VertexPointXY* toEdge = static_cast<VertexPointXY*>(e->vertex(1));
  *(params->os) << fromEdge->estimate().translation().x() << " "
                << fromEdge->estimate().translation().y() << " "
                << fromEdge->estimate().rotation().angle() << std::endl;
  *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y()
                << std::endl;
  *(params->os) << std::endl;
  return this;
}

#ifdef G2O_HAVE_OPENGL
EdgeSE2PointXYBearingDrawAction::EdgeSE2PointXYBearingDrawAction()
    : DrawAction(typeid(EdgeSE2PointXYBearing).name()) {}

HyperGraphElementAction* EdgeSE2PointXYBearingDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  refreshPropertyPtrs(params_);
  if (!_previousParams) return this;

  if (_show && !_show->value()) return this;

  EdgeSE2PointXYBearing* e = static_cast<EdgeSE2PointXYBearing*>(element);
  VertexSE2* from = static_cast<VertexSE2*>(e->vertex(0));
  VertexPointXY* to = static_cast<VertexPointXY*>(e->vertex(1));
  if (!from) return this;
  double guessRange = 5;
  double theta = e->measurement();
  Vector2 p(std::cos(theta) * guessRange, std::sin(theta) * guessRange);
  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
  glDisable(GL_LIGHTING);
  if (!to) {
    p = from->estimate() * p;
    glColor3f(LANDMARK_EDGE_GHOST_COLOR);
    glPushAttrib(GL_POINT_SIZE);
    glPointSize(3);
    glBegin(GL_POINTS);
    glVertex3f((float)p.x(), (float)p.y(), 0.f);
    glEnd();
    glPopAttrib();
  } else {
    p = to->estimate();
    glColor3f(LANDMARK_EDGE_COLOR);
  }
  glBegin(GL_LINES);
  glVertex3f((float)from->estimate().translation().x(),
             (float)from->estimate().translation().y(), 0.f);
  glVertex3f((float)p.x(), (float)p.y(), 0.f);
  glEnd();
  glPopAttrib();
  return this;
}
#endif

}  // namespace g2o
