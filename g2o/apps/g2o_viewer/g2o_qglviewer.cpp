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
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/hyper_graph_action.h"

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

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <iostream>
using namespace std;

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
          return qglv_real(0.001);
        else
          return Camera::zNear();
      }

      qglv_real zFar() const
      {
        if (_standard)
          return qglv_real(10000.0);
        else
          return Camera::zFar();
      }

      bool standard() const {return _standard;}
      void setStandard(bool s) { _standard = s;}

    private:
      bool _standard;
  };

} // end anonymous namespace

G2oQGLViewer::G2oQGLViewer(QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags flags) :
  QGLViewer(parent, shareWidget, flags),
  graph(0), _drawActions(0), _drawList(0), _updateDisplay(true)
{
  setAxisIsDrawn(false);
  _drawActionParameters = new DrawAction::Parameters();
}

G2oQGLViewer::~G2oQGLViewer()
{
  delete _drawActionParameters;
  glDeleteLists(_drawList, 1);
}

void G2oQGLViewer::draw()
{
  if (! graph)
    return;

  if (_drawActions == 0) {
    _drawActions = HyperGraphActionLibrary::instance()->actionByName("draw");
    assert(_drawActions);
  }

  if (! _drawActions) // avoid segmentation fault in release build
    return;
  if (_updateDisplay) {
    _updateDisplay = false;
    glNewList(_drawList, GL_COMPILE_AND_EXECUTE);
    applyAction(graph, _drawActions, _drawActionParameters);
    glEndList();
  } else {
    glCallList(_drawList);
  }
}

void G2oQGLViewer::init()
{
  QGLViewer::init();
  //glDisable(GL_LIGHT0);
 //glDisable(GL_LIGHTING);

  setBackgroundColor(QColor::fromRgb(51, 51, 51));

  // some default settings i like
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  //glEnable(GL_CULL_FACE);
  glShadeModel(GL_FLAT);
  //glShadeModel(GL_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  setAxisIsDrawn();

  // don't save state
  setStateFileName(QString::null);

  // mouse bindings
#ifdef QGLVIEWER_DEPRECATED_MOUSEBINDING
  setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, TRANSLATE);
  setMouseBinding(Qt::NoModifier, Qt::MidButton, CAMERA, TRANSLATE);
#else
  setMouseBinding(Qt::RightButton, CAMERA, TRANSLATE);
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

  // getting a display list
  _drawList = glGenLists(1);
}

void G2oQGLViewer::setUpdateDisplay(bool updateDisplay)
{
  _updateDisplay = updateDisplay;
}

} // end namespace
