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

#include "edge_se2_sensor_calib.h"

#include <string>
#include <typeinfo>

#include "g2o/core/eigen_types.h"
#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif
namespace g2o {

void EdgeSE2SensorCalib::computeError() {
  const VertexSE2* v1 = vertexXnRaw<0>();
  const VertexSE2* v2 = vertexXnRaw<1>();
  const VertexSE2* laserOffset = vertexXnRaw<2>();
  const SE2& x1 = v1->estimate();
  const SE2& x2 = v2->estimate();
  SE2 delta = inverseMeasurement_ * ((x1 * laserOffset->estimate()).inverse() *
                                     x2 * laserOffset->estimate());
  error_ = delta.toVector();
}

void EdgeSE2SensorCalib::setMeasurement(const SE2& m) {
  measurement_ = m;
  inverseMeasurement_ = m.inverse();
}

double EdgeSE2SensorCalib::initialEstimatePossible(
    const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
  if (from.count(vertices_[2]) == 1  // need the laser offset
      && ((from.count(vertices_[0]) == 1 && to == vertices_[1].get()) ||
          ((from.count(vertices_[1]) == 1 && to == vertices_[0].get())))) {
    return 1.0;
  }
  return -1.0;
}

void EdgeSE2SensorCalib::initialEstimate(
    const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
  (void)to;
  auto vi = vertexXn<0>();
  auto vj = vertexXn<1>();
  auto l = vertexXn<2>();
  if (from.count(l) == 0) return;
  if (from.count(vi) == 1) {
    vj->setEstimate(vi->estimate() * l->estimate() * measurement() *
                    l->estimate().inverse());
  } else {
    vi->setEstimate(vj->estimate() * l->estimate() * inverseMeasurement_ *
                    l->estimate().inverse());
  }
}

#ifdef G2O_HAVE_OPENGL
EdgeSE2SensorCalibDrawAction::EdgeSE2SensorCalibDrawAction()
    : DrawAction(typeid(EdgeSE2SensorCalib).name()) {}

bool EdgeSE2SensorCalibDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    HyperGraphElementAction::Parameters&) {
  if (typeid(element).name() != typeName_) return false;
  auto* e = static_cast<EdgeSE2SensorCalib*>(&element);
  auto fromEdge = e->vertexXn<0>();
  auto toEdge = e->vertexXn<1>();
  if (!fromEdge.get() || !toEdge.get()) return true;
  glColor3f(0.5, 0.5, 1.0);
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glVertex3f(static_cast<float>(fromEdge->estimate().translation().x()),
             static_cast<float>(fromEdge->estimate().translation().y()), 0.F);
  glVertex3f(static_cast<float>(toEdge->estimate().translation().x()),
             static_cast<float>(toEdge->estimate().translation().y()), 0.F);
  glEnd();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
