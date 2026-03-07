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

#include "edge_se3_line.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <typeinfo>

#include "g2o/core/parameter.h"
#include "g2o/stuff/opengl_interface.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/types/slam3d_addons/line3d.h"
#include "g2o/types/slam3d_addons/vertex_line3d.h"

namespace g2o {

EdgeSE3Line3D::EdgeSE3Line3D() {
  information().setIdentity();
  resizeParameters(1);
  installParameter<CacheSE3Offset::ParameterType>(0);
  color << 0.0, 0.5, 1.0;
}

void EdgeSE3Line3D::computeError() {
  const VertexSE3* se3Vertex = vertexXnRaw<0>();
  const VertexLine3D* lineVertex = vertexXnRaw<1>();
  const Line3D& line = lineVertex->estimate();
  Line3D localLine = se3Vertex->estimate().inverse() * line;
  error_ = localLine.ominus(measurement_);
}

bool EdgeSE3Line3D::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = parameters_[0];
  cache_ = resolveCache<CacheSE3Offset>(vertexXn<0>(), "CACHE_SE3_OFFSET", pv);
  return cache_ != nullptr;
}

#ifdef G2O_HAVE_OPENGL
// LCOV_EXCL_START
EdgeSE3Line3DDrawAction::EdgeSE3Line3DDrawAction()
    : DrawAction(typeid(EdgeSE3Line3D).name()),
      lineLength_(nullptr),
      lineWidth_(nullptr) {}

DrawAction::Parameters* EdgeSE3Line3DDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters& params_) {
  DrawAction::Parameters* params = DrawAction::refreshPropertyPtrs(params_);
  if (params) {
    lineLength_ =
        params->makeProperty<FloatProperty>(typeName_ + "::LINE_LENGTH", 4.0F);
    lineWidth_ =
        params->makeProperty<FloatProperty>(typeName_ + "::LINE_WIDTH", 2.0F);
  }
  return params;
}

bool EdgeSE3Line3DDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    HyperGraphElementAction::Parameters& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* that = dynamic_cast<EdgeSE3Line3D*>(&element);
  if (!that) return true;
  auto robot = std::dynamic_pointer_cast<VertexSE3>(that->vertex(0));
  auto landmark = std::dynamic_pointer_cast<VertexLine3D>(that->vertex(1));

  if (!robot || !landmark) return false;

  if (lineLength_ && lineWidth_) {
    Line3D line = that->measurement();
    line.normalize();
    Vector3 direction = line.d();
    Vector3 npoint = line.d().cross(line.w());

    g2o::opengl::push_matrix();
    g2o::opengl::mult_matrixd(
        robot->estimate().matrix().cast<double>().eval().data());
    g2o::opengl::color3f(static_cast<float>(that->color(0)),
                         static_cast<float>(that->color(1)),
                         static_cast<float>(that->color(2)));
    g2o::opengl::line_width(static_cast<float>(lineWidth_->value()));
    g2o::opengl::begin_lines();
    g2o::opengl::normal3f(static_cast<float>(npoint.x()),
                          static_cast<float>(npoint.y()),
                          static_cast<float>(npoint.z()));
    g2o::opengl::vertex3f(
        static_cast<float>(npoint.x() -
                           direction.x() * lineLength_->value() / 2),
        static_cast<float>(npoint.y() -
                           direction.y() * lineLength_->value() / 2),
        static_cast<float>(npoint.z() -
                           direction.z() * lineLength_->value() / 2));
    g2o::opengl::vertex3f(
        static_cast<float>(npoint.x() +
                           direction.x() * lineLength_->value() / 2),
        static_cast<float>(npoint.y() +
                           direction.y() * lineLength_->value() / 2),
        static_cast<float>(npoint.z() +
                           direction.z() * lineLength_->value() / 2));
    g2o::opengl::end();
    g2o::opengl::pop_matrix();
  }

  return true;
}
// LCOV_EXCL_STOP
#endif

}  // namespace g2o
