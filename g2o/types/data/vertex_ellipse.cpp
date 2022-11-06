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

#include "vertex_ellipse.h"

#include "g2o/stuff/macros.h"
#include "g2o/stuff/misc.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/EXTERNAL/freeglut/freeglut_minimal.h"
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif

#include <Eigen/Eigenvalues>
#include <iomanip>

namespace g2o {

VertexEllipse::VertexEllipse()
    : covariance_(Matrix3F::Zero()),
      UMatrix_(Matrix2F::Zero()),
      singularValues_(Vector2F::Zero()) {}

void VertexEllipse::updateSVD() const {
  const Eigen::SelfAdjointEigenSolver<Matrix2F> eigenSolver(
      covariance_.block<2, 2>(0, 0));
  UMatrix_ = eigenSolver.eigenvectors();
  singularValues_ = eigenSolver.eigenvalues();
}

bool VertexEllipse::read(std::istream& is) {
  float cxx;
  float cxy;
  float cxt;
  float cyy;
  float cyt;
  float ctt;
  is >> cxx >> cxy >> cxt >> cyy >> cyt >> ctt;
  covariance_(0, 0) = cxx;
  covariance_(0, 1) = cxy;
  covariance_(0, 2) = cxt;
  covariance_(1, 0) = cxy;
  covariance_(1, 1) = cyy;
  covariance_(1, 2) = cyt;
  covariance_(2, 0) = cxt;
  covariance_(2, 1) = cyt;
  covariance_(2, 2) = ctt;

  updateSVD();

  int size;
  is >> size;
  for (int i = 0; i < size; i++) {
    float x;
    float y;
    is >> x >> y;
    addMatchingVertex(x, y);
  }

  return true;
}

bool VertexEllipse::write(std::ostream& os) const {
  os << covariance_(0, 0) << " " << covariance_(0, 1) << " "
     << covariance_(0, 2) << " " << covariance_(1, 1) << " "
     << covariance_(1, 2) << " " << covariance_(2, 2) << " ";

  os << matchingVertices_.size() << " ";
  for (const auto& matchingVertex : matchingVertices_) {
    os << matchingVertex.x() << " " << matchingVertex.y() << " ";
  }

  return os.good();
}

#ifdef G2O_HAVE_OPENGL
VertexEllipseDrawAction::VertexEllipseDrawAction()
    : DrawAction(typeid(VertexEllipse).name()) {
  scaleFactor_ = nullptr;
}

bool VertexEllipseDrawAction::refreshPropertyPtrs(
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (previousParams_) {
    scaleFactor_ =
        previousParams_->makeProperty<DoubleProperty>(typeName_ + "::", 1);
  } else {
    scaleFactor_ = nullptr;
  }
  return true;
}

bool VertexEllipseDrawAction::operator()(
    HyperGraph::HyperGraphElement& element,
    const std::shared_ptr<HyperGraphElementAction::Parameters>& params_) {
  if (typeid(element).name() != typeName_) return false;

  refreshPropertyPtrs(params_);
  if (!previousParams_) {
    return true;
  }
  if (show_ && !show_->value()) return true;

  auto* that = dynamic_cast<VertexEllipse*>(&element);

  glPushMatrix();

  const float sigmaTheta = std::sqrt(that->covariance()(2, 2));
  const float x = 0.1F * cosf(sigmaTheta);
  const float y = 0.1F * sinf(sigmaTheta);

  glColor3f(1.F, 0.7F, 1.F);
  glBegin(GL_LINE_STRIP);
  glVertex3f(x, y, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(x, -y, 0);
  glEnd();

  glColor3f(0.F, 1.F, 0.F);
  for (const auto& i : that->matchingVertices()) {
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(i.x(), i.y(), 0);
    glEnd();
  }

  Matrix2F rot = that->U();
  const float angle = std::atan2(rot(1, 0), rot(0, 0));
  glRotatef(angle * 180.0 / const_pi(), 0., 0., 1.);
  Vector2F sv = that->singularValues();
  glScalef(sqrt(sv(0)), sqrt(sv(1)), 1);

  glColor3f(1.F, 0.7F, 1.F);
  glBegin(GL_LINE_LOOP);
  for (int i = 0; i < 36; i++) {
    const float rad = i * const_pi() / 18.0;
    glVertex2f(std::cos(rad), std::sin(rad));
  }
  glEnd();

  glPopMatrix();
  return true;
}
#endif

}  // namespace g2o
