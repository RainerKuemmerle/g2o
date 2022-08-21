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

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

void EdgeSE2PointXYBearing::initialEstimate(
    const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/) {
  assert(from.size() == 1 && from.count(vertices_[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexPointXY");

  if (from.count(vertices_[0]) != 1) return;
  number_t r = 2.;
  const VertexSE2* v1 = vertexXnRaw<0>();
  VertexPointXY* l2 = vertexXnRaw<1>();
  SE2 t = v1->estimate();
  t.setRotation(t.rotation() * Rotation2D(measurement_));
  Vector2 vr(r, 0.);
  l2->setEstimate(t * vr);
}

bool EdgeSE2PointXYBearing::read(std::istream& is) {
  is >> measurement_ >> information()(0, 0);
  return true;
}

bool EdgeSE2PointXYBearing::write(std::ostream& os) const {
  os << measurement() << " " << information()(0, 0);
  return os.good();
}

EdgeSE2PointXYBearingWriteGnuplotAction::
    EdgeSE2PointXYBearingWriteGnuplotAction()
    : WriteGnuplotAction(typeid(EdgeSE2PointXYBearing).name()) {}

bool EdgeSE2PointXYBearingWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  auto* params = static_cast<WriteGnuplotAction::Parameters*>(params_.get());
  if (!params->os) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified"
              << std::endl;
    return false;
  }

  auto* e = static_cast<EdgeSE2PointXYBearing*>(&element);
  auto fromEdge = e->vertexXn<0>();
  auto toEdge = e->vertexXn<1>();
  *(params->os) << fromEdge->estimate().translation().x() << " "
                << fromEdge->estimate().translation().y() << " "
                << fromEdge->estimate().rotation().angle() << std::endl;
  *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y()
                << std::endl;
  *(params->os) << std::endl;
  return true;
}

#ifdef G2O_HAVE_OPENGL
EdgeSE2PointXYBearingDrawAction::EdgeSE2PointXYBearingDrawAction()
    : DrawAction(typeid(EdgeSE2PointXYBearing).name()) {}

bool EdgeSE2PointXYBearingDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* e = static_cast<EdgeSE2PointXYBearing*>(&element);
  auto from = e->vertexXn<0>();
  auto to = e->vertexXn<1>();
  if (!from) return true;
  number_t guessRange = 5;
  number_t theta = e->measurement();
  Vector2 p(std::cos(theta) * guessRange, std::sin(theta) * guessRange);
  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
  glDisable(GL_LIGHTING);
  if (!to) {
    p = from->estimate() * p;
    glColor3f(LANDMARK_EDGE_GHOST_COLOR);
    glPushAttrib(GL_POINT_SIZE);
    glPointSize(3);
    glBegin(GL_POINTS);
    glVertex3f(static_cast<float>(p.x()), static_cast<float>(p.y()), 0.F);
    glEnd();
    glPopAttrib();
  } else {
    p = to->estimate();
    glColor3f(LANDMARK_EDGE_COLOR);
  }
  glBegin(GL_LINES);
  glVertex3f(static_cast<float>(from->estimate().translation().x()),
             static_cast<float>(from->estimate().translation().y()), 0.F);
  glVertex3f(static_cast<float>(p.x()), static_cast<float>(p.y()), 0.F);
  glEnd();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
