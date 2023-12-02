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

#include "g2o_qglviewer.h"

#include <cassert>

#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/sparse_optimizer.h"

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

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
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
    if (standard_) return qglv_real(0.001);
    return Camera::zNear();
  }

  qglv_real zFar() const override {
    if (standard_) return qglv_real(10000.0);
    return Camera::zFar();
  }

  bool standard() const { return standard_; }
  void setStandard(bool s) { standard_ = s; }

 private:
  bool standard_ = true;
};

}  // end anonymous namespace

G2oQGLViewer::G2oQGLViewer(QWidget* parent)
    : QGLViewer(parent), drawActions_(nullptr) {
  setAxisIsDrawn(false);
  drawActionParameters_ = std::make_shared<DrawAction::Parameters>();
}

G2oQGLViewer::~G2oQGLViewer() { glDeleteLists(drawList_, 1); }

void G2oQGLViewer::draw() {
  if (!graph) return;

  if (drawActions_ == nullptr) {
    drawActions_ = HyperGraphActionLibrary::instance()->actionByName("draw");
    assert(drawActions_);
  }

  if (!drawActions_)  // avoid segmentation fault in release build
    return;
  if (updateDisplay_) {
    updateDisplay_ = false;
    glNewList(drawList_, GL_COMPILE_AND_EXECUTE);
    SparseOptimizer& optimizer = *graph;
    applyAction(optimizer, *drawActions_, drawActionParameters_);
    glEndList();
  } else {
    glCallList(drawList_);
  }
}

void G2oQGLViewer::init() {
  QGLViewer::init();
  // glDisable(GL_LIGHT0);
  // glDisable(GL_LIGHTING);

  setBackgroundColor(QColor::fromRgb(51, 51, 51));

  // some default settings i like
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  // glEnable(GL_CULL_FACE);
  glShadeModel(GL_FLAT);
  // glShadeModel(GL_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setAxisIsDrawn();

  // don't save state
  setStateFileName(QString());

  // mouse bindings
#ifdef QGLVIEWER_DEPRECATED_MOUSEBINDING
  setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, TRANSLATE);
  setMouseBinding(Qt::NoModifier, Qt::MiddleButton, CAMERA, TRANSLATE);
#else
  setMouseBinding(Qt::RightButton, CAMERA, TRANSLATE);
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

  // getting a display list
  drawList_ = glGenLists(1);
}

void G2oQGLViewer::setUpdateDisplay(bool updateDisplay) {
  updateDisplay_ = updateDisplay;
}

}  // namespace g2o
