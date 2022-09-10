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

#include "edge_se3_plane_calib.h"

#include <iostream>

#include "g2o/stuff/opengl_wrapper.h"

namespace g2o {

EdgeSE3PlaneSensorCalib::EdgeSE3PlaneSensorCalib()
    : color(cst(0.1), cst(0.1), cst(0.1)) {
  resize(3);
}

bool EdgeSE3PlaneSensorCalib::read(std::istream& is) {
  Vector4 v;
  bool state = internal::readVector(is, v);
  setMeasurement(Plane3D(v));
  state &= internal::readVector(is, color);
  state &= readInformationMatrix(is);
  return state;
}

bool EdgeSE3PlaneSensorCalib::write(std::ostream& os) const {
  internal::writeVector(os, measurement().toVector());
  internal::writeVector(os, color);
  return writeInformationMatrix(os);
}

#ifdef G2O_HAVE_OPENGL
EdgeSE3PlaneSensorCalibDrawAction::EdgeSE3PlaneSensorCalibDrawAction()
    : DrawAction(typeid(EdgeSE3PlaneSensorCalib).name()),
      planeWidth_(nullptr),
      planeHeight_(nullptr) {}

bool EdgeSE3PlaneSensorCalibDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    planeWidth_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::PLANE_WIDTH", 0.5F);
    planeHeight_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::PLANE_HEIGHT", 0.5F);
  } else {
    planeWidth_ = nullptr;
    planeHeight_ = nullptr;
  }
  return true;
}

bool EdgeSE3PlaneSensorCalibDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* that = dynamic_cast<EdgeSE3PlaneSensorCalib*>(&element);

  if (!that) return true;

  const VertexSE3* robot =
      dynamic_cast<const VertexSE3*>(that->vertex(0).get());
  const VertexSE3* sensor =
      dynamic_cast<const VertexSE3*>(that->vertex(2).get());
  if (!robot || !sensor) return false;

  if (planeWidth_ && planeHeight_) {
    number_t d = that->measurement().distance();
    number_t azimuth = Plane3D::azimuth(that->measurement().normal());
    number_t elevation = Plane3D::elevation(that->measurement().normal());

    glColor3f(static_cast<float>(that->color(0)),
              static_cast<float>(that->color(1)),
              static_cast<float>(that->color(2)));
    glPushMatrix();
    Isometry3 robotAndSensor = robot->estimate() * sensor->estimate();
    glMultMatrixd(robotAndSensor.matrix().cast<double>().eval().data());

    glRotatef(static_cast<float>(RAD2DEG(azimuth)), 0.F, 0.F, 1.F);
    glRotatef(static_cast<float>(RAD2DEG(elevation)), 0.F, -1.F, 0.F);
    glTranslatef(static_cast<float>(d), 0.F, 0.F);

    float planeWidth = planeWidth_->value();
    float planeHeight = planeHeight_->value();
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glVertex3f(0, -planeWidth, -planeHeight);
    glVertex3f(0, planeWidth, -planeHeight);
    glVertex3f(0, planeWidth, planeHeight);
    glVertex3f(0, -planeWidth, planeHeight);
    glEnd();
    glPopMatrix();
  }

  return true;
}
#endif

}  // namespace g2o
