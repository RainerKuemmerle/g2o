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

#include "edge_se2_odom_differential_calib.h"

#include <string>

#include "g2o/types/sclam2d/odometry_measurement.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

void EdgeSE2OdomDifferentialCalib::computeError() {
  const VertexSE2* v1 = vertexXnRaw<0>();
  const VertexSE2* v2 = vertexXnRaw<1>();
  const VertexOdomDifferentialParams* params = vertexXnRaw<2>();
  const SE2& x1 = v1->estimate();
  const SE2& x2 = v2->estimate();

  // get the calibrated motion given by the odometry
  VelocityMeasurement calibratedVelocityMeasurement(
      measurement().vl() * params->estimate()(0),
      measurement().vr() * params->estimate()(1), measurement().dt());
  MotionMeasurement mm = OdomConvert::convertToMotion(
      calibratedVelocityMeasurement, params->estimate()(2));
  SE2 Ku_ij;
  Ku_ij.fromVector(mm.measurement());

  SE2 delta = Ku_ij.inverse() * x1.inverse() * x2;
  error_ = delta.toVector();
}

#ifdef G2O_HAVE_OPENGL
EdgeSE2OdomDifferentialCalibDrawAction::EdgeSE2OdomDifferentialCalibDrawAction()
    : DrawAction(typeid(EdgeSE2OdomDifferentialCalib).name()) {}

bool EdgeSE2OdomDifferentialCalibDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>&) {
  if (typeid(element).name() != typeName_) return false;
  auto* e = static_cast<EdgeSE2OdomDifferentialCalib*>(&element);
  auto fromEdge = e->vertexXn<0>();
  auto toEdge = e->vertexXn<1>();
  if (!fromEdge || !toEdge) return true;
  glColor3f(0.5F, 0.5F, 0.5F);
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
