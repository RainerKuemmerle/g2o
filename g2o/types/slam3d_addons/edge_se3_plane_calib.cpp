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

#include <string>

#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/opengl_interface.h"

namespace g2o {

EdgeSE3PlaneSensorCalib::EdgeSE3PlaneSensorCalib()
    : color(cst(0.1), cst(0.1), cst(0.1)) {
  resize(3);
}

#ifdef G2O_HAVE_OPENGL
// LCOV_EXCL_START
EdgeSE3PlaneSensorCalibDrawAction::EdgeSE3PlaneSensorCalibDrawAction()
    : DrawAction(typeid(EdgeSE3PlaneSensorCalib).name()),
      planeWidth_(nullptr),
      planeHeight_(nullptr) {}

DrawAction::Parameters* EdgeSE3PlaneSensorCalibDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters& params_) {
  DrawAction::Parameters* params = DrawAction::refreshPropertyPtrs(params_);
  if (params) {
    planeWidth_ =
        params->makeProperty<FloatProperty>(typeName_ + "::PLANE_WIDTH", 0.5F);
    planeHeight_ =
        params->makeProperty<FloatProperty>(typeName_ + "::PLANE_HEIGHT", 0.5F);
  }
  return params;
}

bool EdgeSE3PlaneSensorCalibDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    HyperGraphElementAction::Parameters& params_) {
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
    double d = that->measurement().distance();
    double azimuth = Plane3D::azimuth(that->measurement().normal());
    double elevation = Plane3D::elevation(that->measurement().normal());

    g2o::opengl::color3f(static_cast<float>(that->color(0)),
                         static_cast<float>(that->color(1)),
                         static_cast<float>(that->color(2)));
    g2o::opengl::push_matrix();
    Isometry3 robotAndSensor = robot->estimate() * sensor->estimate();
    g2o::opengl::mult_matrixd(
        robotAndSensor.matrix().cast<double>().eval().data());

    g2o::opengl::rotatef(static_cast<float>(RAD2DEG(azimuth)), 0.F, 0.F, 1.F);
    g2o::opengl::rotatef(static_cast<float>(RAD2DEG(elevation)), 0.F, -1.F,
                         0.F);
    g2o::opengl::translatef(static_cast<float>(d), 0.F, 0.F);

    float planeWidth = planeWidth_->value();
    float planeHeight = planeHeight_->value();
    g2o::opengl::begin_quads();
    g2o::opengl::normal3f(-1, 0, 0);
    g2o::opengl::vertex3f(0, -planeWidth, -planeHeight);
    g2o::opengl::vertex3f(0, planeWidth, -planeHeight);
    g2o::opengl::vertex3f(0, planeWidth, planeHeight);
    g2o::opengl::vertex3f(0, -planeWidth, planeHeight);
    g2o::opengl::end();
    g2o::opengl::pop_matrix();
  }

  return true;
}
// LCOV_EXCL_STOP
#endif

}  // namespace g2o
