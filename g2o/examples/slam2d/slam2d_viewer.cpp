// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#include "slam2d_viewer.h"

#include <Eigen/Core>
#include <iostream>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"

// some macro helpers for identifying the version number of QGLViewer
// QGLViewer changed some parts of its API in version 2.6.
// The following preprocessor hack accounts for this. THIS SUCKS!!!
#if (((QGLVIEWER_VERSION & 0xff0000) >> 16) >= 2 && \
     ((QGLVIEWER_VERSION & 0x00ff00) >> 8) >= 6)
#define qglv_real qreal
#else
#define qglv_real float
#endif

// Again, some API changes in QGLViewer which produce annoying text in the
// console if the old API is used.
#if (((QGLVIEWER_VERSION & 0xff0000) >> 16) >= 2 && \
     ((QGLVIEWER_VERSION & 0x00ff00) >> 8) >= 5)
#define QGLVIEWER_DEPRECATED_MOUSEBINDING
#endif

namespace g2o {

namespace {

/**
 * \brief helper for setting up a camera for qglviewer
 */
class StandardCamera : public qglviewer::Camera {
 public:
  StandardCamera() = default;

  qglv_real zNear() const override {
    if (standard_) return 0.001F;
    return Camera::zNear();
  }

  qglv_real zFar() const override {
    if (standard_) return 1000.0F;
    return Camera::zFar();
  }

  const bool& standard() const { return standard_; }
  bool& standard() { return standard_; }

 private:
  bool standard_ = true;
};

void drawSE2(const VertexSE2* v) {
  constexpr double kLen = 0.2;
  static const Eigen::Vector2d kP1(0.75 * kLen, 0.);
  static const Eigen::Vector2d kP2(-0.25 * kLen, 0.5 * kLen);
  static const Eigen::Vector2d kP3(-0.25 * kLen, -0.5 * kLen);

  const SE2& pose = v->estimate();

  Eigen::Vector2d aux = pose * kP1;
  glVertex3f(aux[0], aux[1], 0.F);
  aux = pose * kP2;
  glVertex3f(aux[0], aux[1], 0.F);
  aux = pose * kP3;
  glVertex3f(aux[0], aux[1], 0.F);
}

template <typename Derived>
void drawCov(const Eigen::Vector2d& p, const Eigen::MatrixBase<Derived>& cov) {
  const double scalingFactor = 1.;

  glPushMatrix();
  glTranslatef(p.x(), p.y(), 0.F);

  const typename Derived::Scalar& a = cov(0, 0);
  const typename Derived::Scalar& b = cov(0, 1);
  const typename Derived::Scalar& d = cov(1, 1);

  /* get eigen-values */
  double D = a * d - b * b;  // determinant of the matrix
  double T = a + d;          // Trace of the matrix
  const double h = sqrt(0.25 * (T * T) - D);
  const double lambda1 =
      0.5 * T + h;  // solving characteristic polynom using p-q-formula
  const double lambda2 = 0.5 * T - h;

  const double theta = 0.5 * atan2(2.0 * b, a - d);
  const double majorAxis = 3.0 * sqrt(lambda1);
  const double minorAxis = 3.0 * sqrt(lambda2);

  glRotatef(RAD2DEG(theta), 0.F, 0.F, 1.F);
  glScalef(majorAxis * scalingFactor, minorAxis * scalingFactor, 1.F);
  glColor4f(1.0F, 1.F, 0.F, 0.4F);
  opengl::drawDisk(1.F);
  glColor4f(0.F, 0.F, 0.F, 1.0F);
  opengl::drawCircle(1.F);
  glPopMatrix();
}

}  // end anonymous namespace

Slam2DViewer::Slam2DViewer(QWidget* parent, const QGLWidget* shareWidget)
    : QGLViewer(parent, shareWidget), graph(nullptr) {}

void Slam2DViewer::draw() {
  if (!graph) return;

  // drawing the graph
  glColor4f(0.00F, 0.67F, 1.00F, 1.F);
  glBegin(GL_TRIANGLES);
  for (auto& v_ptr : graph->vertices()) {
    auto* v = dynamic_cast<VertexSE2*>(v_ptr.second.get());
    if (v) {
      drawSE2(v);
    }
  }
  glEnd();

  glColor4f(1.00F, 0.67F, 0.00F, 1.F);
  glPointSize(2.F);
  glBegin(GL_POINTS);
  for (auto& v_ptr : graph->vertices()) {
    auto* v = dynamic_cast<VertexPointXY*>(v_ptr.second.get());
    if (v) {
      glVertex3f(v->estimate()(0), v->estimate()(1), 0.F);
    }
  }
  glEnd();
  glPointSize(1.F);

  if (drawCovariance) {
    for (auto& v_ptr : graph->vertices()) {
      auto* v = dynamic_cast<VertexSE2*>(v_ptr.second.get());
      if (v) {
        // TODO(Rainer): Implement draw function for covariance.
        // drawCov(v->estimate().translation(), v->uncertainty());
      }
    }
  }
}

void Slam2DViewer::init() {
  QGLViewer::init();

  // some default settings i like
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glShadeModel(GL_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setAxisIsDrawn();

  // don't save state
  setStateFileName(QString());

  // mouse bindings
#ifdef QGLVIEWER_DEPRECATED_MOUSEBINDING
  setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, ZOOM);
  setMouseBinding(Qt::NoModifier, Qt::MidButton, CAMERA, TRANSLATE);
#else
  setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
  setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);
#endif

  // keyboard shortcuts
  setShortcut(CAMERA_MODE, 0);
  setShortcut(EXIT_VIEWER, 0);
  // setShortcut(SAVE_SCREENSHOT, 0);

  // replace camera
  qglviewer::Camera* oldcam = camera();
  qglviewer::Camera* cam = new StandardCamera();
  setCamera(cam);
  cam->setPosition(qglviewer::Vec(0., 0., 75.));
  cam->setUpVector(qglviewer::Vec(0., 1., 0.));
  cam->lookAt(qglviewer::Vec(0., 0., 0.));
  delete oldcam;
}

}  // namespace g2o
