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

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <typeinfo>

#include "g2o/stuff/macros.h"

namespace g2o {

VertexSegment2D::VertexSegment2D()

{
  estimate_.setZero();
}

bool VertexSegment2D::read(std::istream& is) {
  return internal::readVector(is, estimate_);
}

bool VertexSegment2D::write(std::ostream& os) const {
  return internal::writeVector(os, estimate());
}

VertexSegment2DWriteGnuplotAction::VertexSegment2DWriteGnuplotAction()
    : WriteGnuplotAction(typeid(VertexSegment2D).name()) {}

bool VertexSegment2DWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  auto* params = static_cast<WriteGnuplotAction::Parameters*>(params_.get());
  if (!params->os) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified"
              << std::endl;
    return false;
  }

  auto* v = static_cast<VertexSegment2D*>(&element);
  *(params->os) << v->estimateP1().x() << " " << v->estimateP1().y()
                << std::endl;
  *(params->os) << v->estimateP2().x() << " " << v->estimateP2().y()
                << std::endl;
  *(params->os) << std::endl;
  return true;
}

#ifdef G2O_HAVE_OPENGL
VertexSegment2DDrawAction::VertexSegment2DDrawAction()
    : DrawAction(typeid(VertexSegment2D).name()), pointSize_(nullptr) {}

bool VertexSegment2DDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    pointSize_ = previousParams_->makeProperty<FloatProperty>(
        typeName_ + "::POINT_SIZE", 1.);
  } else {
    pointSize_ = nullptr;
  }
  return true;
}

bool VertexSegment2DDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* that = static_cast<VertexSegment2D*>(&element);
  glColor3f(0.8F, 0.5F, 0.3F);
  if (pointSize_) {
    glPointSize(pointSize_->value());
  }
  glBegin(GL_LINES);
  glVertex3f(static_cast<float>(that->estimateP1().x()),
             static_cast<float>(that->estimateP1().y()), 0.F);
  glVertex3f(static_cast<float>(that->estimateP2().x()),
             static_cast<float>(that->estimateP2().y()), 0.F);
  glEnd();
  return true;
}
#endif

}  // namespace g2o
