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

#include "vertex_line2d.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#endif

namespace g2o {

VertexLine2D::VertexLine2D() { estimate_.setZero(); }

bool VertexLine2D::read(std::istream& is) {
  is >> estimate_[0] >> estimate_[1] >> p1Id >> p2Id;
  return true;
}

bool VertexLine2D::write(std::ostream& os) const {
  os << estimate()(0) << " " << estimate()(1) << " " << p1Id << " " << p2Id;
  return os.good();
}

#ifdef G2O_HAVE_OPENGL
VertexLine2DDrawAction::VertexLine2DDrawAction()
    : DrawAction(typeid(VertexLine2D).name()), pointSize_(nullptr) {}

bool VertexLine2DDrawAction::refreshPropertyPtrs(
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

bool VertexLine2DDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) return true;

  if (show_ && !show_->value()) return true;

  auto* that = static_cast<VertexLine2D*>(&element);
  glPushAttrib(GL_CURRENT_BIT | GL_BLEND);
  if (pointSize_) {
    glPointSize(pointSize_->value());
  }
  Vector2 n(std::cos(that->theta()), std::sin(that->theta()));
  Vector2 pmiddle = n * that->rho();
  Vector2 t(-n.y(), n.x());
  number_t l1{};
  number_t l2 = 10;
  auto vp1 = std::dynamic_pointer_cast<VertexPointXY>(
      that->graph()->vertex(that->p1Id));
  auto vp2 = std::dynamic_pointer_cast<VertexPointXY>(
      that->graph()->vertex(that->p2Id));

  glColor4f(0.8F, 0.5F, 0.3F, 0.3F);
  if (vp1 && vp2) {
    glColor4f(0.8F, 0.5F, 0.3F, 0.7F);
  } else if (vp1 || vp2) {
    glColor4f(0.8F, 0.5F, 0.3F, 0.5F);
  }

  if (vp1) {
    glColor4f(0.8F, 0.5F, 0.3F, 0.7F);
    l1 = t.dot(vp1->estimate() - pmiddle);
  }
  if (vp2) {
    glColor4f(0.8F, 0.5F, 0.3F, 0.7F);
    l2 = t.dot(vp2->estimate() - pmiddle);
  }
  Vector2 p1 = pmiddle + t * l1;
  Vector2 p2 = pmiddle + t * l2;
  glBegin(GL_LINES);
  glVertex3f(static_cast<float>(p1.x()), p1.y(), 0.F);
  glVertex3f(static_cast<float>(p2.x()), p2.y(), 0.F);
  glEnd();
  glPopAttrib();
  return true;
}
#endif

}  // namespace g2o
