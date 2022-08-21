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

#include "edge_se2_pointxy.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

bool EdgeSE2PointXY::read(std::istream& is) {
  internal::readVector(is, measurement_);
  readInformationMatrix(is);
  return true;
}

bool EdgeSE2PointXY::write(std::ostream& os) const {
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE2PointXY::initialEstimate(const OptimizableGraph::VertexSet& from,
                                     OptimizableGraph::Vertex* to) {
  assert(from.size() == 1 && from.count(vertices_[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexPointXY");

  auto vi = vertexXn<0>();
  auto vj = vertexXn<1>();
  if (from.count(vi) > 0 && to == vj.get()) {
    vj->setEstimate(vi->estimate() * measurement_);
  }
}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgeSE2PointXY::linearizeOplus() {
  const VertexSE2* vi = vertexXnRaw<0>();
  const VertexPointXY* vj = vertexXnRaw<1>();
  const number_t& x1 = vi->estimate().translation()[0];
  const number_t& y1 = vi->estimate().translation()[1];
  const number_t& th1 = vi->estimate().rotation().angle();
  const number_t& x2 = vj->estimate()[0];
  const number_t& y2 = vj->estimate()[1];

  number_t aux_1 = std::cos(th1);
  number_t aux_2 = -aux_1;
  number_t aux_3 = std::sin(th1);

  jacobianOplusXi_(0, 0) = aux_2;
  jacobianOplusXi_(0, 1) = -aux_3;
  jacobianOplusXi_(0, 2) = aux_1 * y2 - aux_1 * y1 - aux_3 * x2 + aux_3 * x1;
  jacobianOplusXi_(1, 0) = aux_3;
  jacobianOplusXi_(1, 1) = aux_2;
  jacobianOplusXi_(1, 2) = -aux_3 * y2 + aux_3 * y1 - aux_1 * x2 + aux_1 * x1;

  jacobianOplusXj_(0, 0) = aux_1;
  jacobianOplusXj_(0, 1) = aux_3;
  jacobianOplusXj_(1, 0) = -aux_3;
  jacobianOplusXj_(1, 1) = aux_1;
}
#endif

EdgeSE2PointXYWriteGnuplotAction::EdgeSE2PointXYWriteGnuplotAction()
    : WriteGnuplotAction(typeid(EdgeSE2PointXY).name()) {}

bool EdgeSE2PointXYWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;
  auto* params = static_cast<WriteGnuplotAction::Parameters*>(params_.get());
  if (!params->os) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified"
              << std::endl;
    return false;
  }

  auto* e = static_cast<EdgeSE2PointXY*>(&element);
  if (e->numUndefinedVertices()) return true;
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
EdgeSE2PointXYDrawAction::EdgeSE2PointXYDrawAction()
    : DrawAction(typeid(EdgeSE2PointXY).name()) {}

bool EdgeSE2PointXYDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* e = static_cast<EdgeSE2PointXY*>(&element);
  auto fromEdge = e->vertexXn<0>();
  auto toEdge = e->vertexXn<1>();
  if (!fromEdge) return true;
  Vector2 p = e->measurement();
  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
  glDisable(GL_LIGHTING);
  if (!toEdge) {
    p = fromEdge->estimate() * p;
    glColor3f(LANDMARK_EDGE_GHOST_COLOR);
    glPushAttrib(GL_POINT_SIZE);
    glPointSize(3);
    glBegin(GL_POINTS);
    glVertex3f(static_cast<float>(p.x()), static_cast<float>(p.y()), 0.F);
    glEnd();
    glPopAttrib();
  } else {
    p = toEdge->estimate();
    glColor3f(LANDMARK_EDGE_COLOR);
  }
  glBegin(GL_LINES);
  glVertex3f(static_cast<float>(fromEdge->estimate().translation().x()),
             static_cast<float>(fromEdge->estimate().translation().y()), 0.F);
  glVertex3f(static_cast<float>(p.x()), static_cast<float>(p.y()), 0.F);
  glEnd();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
