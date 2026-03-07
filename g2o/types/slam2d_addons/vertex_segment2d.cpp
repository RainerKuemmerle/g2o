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

#include "vertex_segment2d.h"

#include "g2o/core/hyper_graph_action.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_interface.h"
#endif

#include <string>
#include <typeinfo>

namespace g2o {

VertexSegment2D::VertexSegment2D() { estimate_.setZero(); }

#ifdef G2O_HAVE_OPENGL
// LCOV_EXCL_START
VertexSegment2DDrawAction::VertexSegment2DDrawAction()
    : DrawAction(typeid(VertexSegment2D).name()), pointSize_(nullptr) {}

DrawAction::Parameters* VertexSegment2DDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters& params_) {
  DrawAction::Parameters* params = DrawAction::refreshPropertyPtrs(params_);
  if (params) {
    pointSize_ =
        params->makeProperty<FloatProperty>(typeName_ + "::POINT_SIZE", 1.);
  }
  return params;
}

bool VertexSegment2DDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    HyperGraphElementAction::Parameters& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* that = static_cast<VertexSegment2D*>(&element);
  g2o::opengl::color3f(0.8F, 0.5F, 0.3F);
  if (pointSize_) {
    g2o::opengl::point_size(pointSize_->value());
  }
  g2o::opengl::begin_lines();
  g2o::opengl::vertex3f(static_cast<float>(that->estimateP1().x()),
                        static_cast<float>(that->estimateP1().y()), 0.F);
  g2o::opengl::vertex3f(static_cast<float>(that->estimateP2().x()),
                        static_cast<float>(that->estimateP2().y()), 0.F);
  g2o::opengl::end();
  return true;
}
// LCOV_EXCL_STOP
#endif

}  // namespace g2o
