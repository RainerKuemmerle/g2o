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
using namespace std;
using namespace Eigen;

EdgeSE3PlaneSensorCalib::EdgeSE3PlaneSensorCalib() : BaseMultiEdge<3, Plane3D>(), color(cst(0.1), cst(0.1), cst(0.1)) {
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
    : DrawAction(typeid(EdgeSE3PlaneSensorCalib).name()), _planeWidth(nullptr), _planeHeight(nullptr) {}

bool EdgeSE3PlaneSensorCalibDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams) {
    _planeWidth = _previousParams->makeProperty<FloatProperty>(_typeName + "::PLANE_WIDTH", 0.5f);
    _planeHeight = _previousParams->makeProperty<FloatProperty>(_typeName + "::PLANE_HEIGHT", 0.5f);
  } else {
    _planeWidth = 0;
    _planeHeight = 0;
  }
  return true;
}

HyperGraphElementAction* EdgeSE3PlaneSensorCalibDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                                                                       HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  refreshPropertyPtrs(params_);
  if (!_previousParams) return this;

  if (_show && !_show->value()) return this;

  EdgeSE3PlaneSensorCalib* that = dynamic_cast<EdgeSE3PlaneSensorCalib*>(element);

  if (!that) return this;

  const VertexSE3* robot = dynamic_cast<const VertexSE3*>(that->vertex(0));
  const VertexSE3* sensor = dynamic_cast<const VertexSE3*>(that->vertex(2));
  if (!robot || !sensor) return nullptr;

  if (_planeWidth && _planeHeight) {
    number_t d = that->measurement().distance();
    number_t azimuth = Plane3D::azimuth(that->measurement().normal());
    number_t elevation = Plane3D::elevation(that->measurement().normal());

    glColor3f(float(that->color(0)), float(that->color(1)), float(that->color(2)));
    glPushMatrix();
    Isometry3 robotAndSensor = robot->estimate() * sensor->estimate();
    glMultMatrixd(robotAndSensor.matrix().cast<double>().eval().data());

    glRotatef(float(RAD2DEG(azimuth)), 0.f, 0.f, 1.f);
    glRotatef(float(RAD2DEG(elevation)), 0.f, -1.f, 0.f);
    glTranslatef(float(d), 0.f, 0.f);

    float planeWidth = _planeWidth->value();
    float planeHeight = _planeHeight->value();
    glBegin(GL_QUADS);
    glNormal3f(-1, 0, 0);
    glVertex3f(0, -planeWidth, -planeHeight);
    glVertex3f(0, planeWidth, -planeHeight);
    glVertex3f(0, planeWidth, planeHeight);
    glVertex3f(0, -planeWidth, planeHeight);
    glEnd();
    glPopMatrix();
  }

  return this;
}
#endif

}  // namespace g2o
