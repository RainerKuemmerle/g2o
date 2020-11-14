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

#include "draw_helpers.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/core/sparse_optimizer.h"

#include <Eigen/Core>
#include <iostream>
using namespace std;

// some macro helpers for identifying the version number of QGLViewer
// QGLViewer changed some parts of its API in version 2.6.
// The following preprocessor hack accounts for this. THIS SUCKS!!!
#if (((QGLVIEWER_VERSION & 0xff0000) >> 16) >= 2 && ((QGLVIEWER_VERSION & 0x00ff00) >> 8) >= 6)
#define qglv_real qreal
#else
#define qglv_real float
#endif

// Again, some API changes in QGLViewer which produce annoying text in the console
// if the old API is used.
#if (((QGLVIEWER_VERSION & 0xff0000) >> 16) >= 2 && ((QGLVIEWER_VERSION & 0x00ff00) >> 8) >= 5)
#define QGLVIEWER_DEPRECATED_MOUSEBINDING
#endif

namespace g2o {

namespace {

  /**
   * \brief helper for setting up a camera for qglviewer
   */
  class StandardCamera : public qglviewer::Camera
  {
    public:
      StandardCamera() : _standard(true) {};

      qglv_real zNear() const {
        if (_standard)
          return 0.001f;
        else
          return Camera::zNear();
      }

      qglv_real zFar() const
      {
        if (_standard)
          return 1000.0f;
        else
          return Camera::zFar();
      }

      const bool& standard() const {return _standard;}
      bool& standard() {return _standard;}

    private:
      bool _standard;
  };

  void drawSE2(const VertexSE2* v)
  {
    static const double len = 0.2;
    static Eigen::Vector2d p1( 0.75 * len,  0.);
    static Eigen::Vector2d p2(-0.25 * len,  0.5 * len);
    static Eigen::Vector2d p3(-0.25 * len, -0.5 * len);

    const SE2& pose = v->estimate();

    Eigen::Vector2d aux = pose * p1;
    glVertex3f(aux[0], aux[1], 0.f);
    aux = pose * p2;
    glVertex3f(aux[0], aux[1], 0.f);
    aux = pose * p3;
    glVertex3f(aux[0], aux[1], 0.f);
  }

  template <typename Derived>
  void drawCov(const Eigen::Vector2d& p, const Eigen::MatrixBase<Derived>& cov)
  {
    const double scalingFactor = 1.;

    glPushMatrix();
    glTranslatef(p.x(), p.y(), 0.f);

    const typename Derived::Scalar& a = cov(0, 0);
    const typename Derived::Scalar& b = cov(0, 1);
    const typename Derived::Scalar& d = cov(1, 1);

    /* get eigen-values */
    double D = a*d - b*b; // determinant of the matrix
    double T = a+d;       // Trace of the matrix
    double h = sqrt(0.25*(T*T) - D);
    double lambda1 = 0.5*T + h;  // solving characteristic polynom using p-q-formula
    double lambda2 = 0.5*T - h;

    double theta     = 0.5 * atan2(2.0 * b, a - d);
    double majorAxis = 3.0 * sqrt(lambda1);
    double minorAxis = 3.0 * sqrt(lambda2);


    glRotatef(RAD2DEG(theta), 0.f, 0.f, 1.f);
    glScalef(majorAxis * scalingFactor, minorAxis * scalingFactor, 1.f);
    glColor4f(1.0f, 1.f, 0.f, 0.4f);
    drawDisk(1.f);
    glColor4f(0.f, 0.f, 0.f, 1.0f);
    drawCircle(1.f);
    glPopMatrix();
  }

} // end anonymous namespace

Slam2DViewer::Slam2DViewer(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags flags) :
  QGLViewer(parent, shareWidget, flags),
  graph(0), drawCovariance(false)
{
}

Slam2DViewer::~Slam2DViewer()
{
}

void Slam2DViewer::draw()
{
  if (! graph)
    return;

  // drawing the graph
  glColor4f(0.00f, 0.67f, 1.00f, 1.f);
  glBegin(GL_TRIANGLES);
  for (SparseOptimizer::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
    VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
    if (v) {
      drawSE2(v);
    }
  }
  glEnd();

  glColor4f(1.00f, 0.67f, 0.00f, 1.f);
  glPointSize(2.f);
  glBegin(GL_POINTS);
  for (SparseOptimizer::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
    VertexPointXY* v = dynamic_cast<VertexPointXY*>(it->second);
    if (v) {
      glVertex3f(v->estimate()(0), v->estimate()(1), 0.f);
    }
  }
  glEnd();
  glPointSize(1.f);

  if (drawCovariance) {
    for (SparseOptimizer::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
      VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
      if (v) {
        // TODO
        //drawCov(v->estimate().translation(), v->uncertainty());
      }
    }
  }
}

void Slam2DViewer::init()
{
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
  //setShortcut(SAVE_SCREENSHOT, 0);

  // replace camera
  qglviewer::Camera* oldcam = camera();
  qglviewer::Camera* cam = new StandardCamera();
  setCamera(cam);
  cam->setPosition(qglviewer::Vec(0., 0., 75.));
  cam->setUpVector(qglviewer::Vec(0., 1., 0.));
  cam->lookAt(qglviewer::Vec(0., 0., 0.));
  delete oldcam;
}

} // end namespace
