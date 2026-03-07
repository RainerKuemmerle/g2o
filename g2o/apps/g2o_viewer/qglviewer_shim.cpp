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

#include "qglviewer_shim.h"

#include <QImage>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QQuaternion>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

#include "g2o/stuff/logger.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/opengl_interface.h"

#if G2O_HAVE_JSON
#include <nlohmann/json.hpp>
#endif

namespace g2o::viewer {

namespace {
constexpr double kFovDegrees = 45.0;

inline float unitsPerPixelForDistance(float dist, int viewportH) {
  constexpr float kFov = g2o::deg2rad(kFovDegrees);
  const float worldHeight = 2.0F * dist * std::tan(kFov * 0.5F);
  return viewportH > 0 ? (worldHeight / static_cast<float>(viewportH))
                       : 0.01F * dist;
}

}  // namespace

QGLViewerShim::QGLViewerShim(QWidget* parent) : QOpenGLWidget(parent) {
  QSurfaceFormat fmt;
  fmt.setRenderableType(QSurfaceFormat::OpenGL);
  fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
  fmt.setVersion(2, 1);
  setFormat(fmt);
  // default camera: positioned at (0,0,10) looking at origin with Y up
  camera_.setTarget(Vec(0, 0, 0));
  camera_.setPosition(Vec(0, 0, 10));
  camera_.setUpVector(Vec(0, 1, 0));
}

QGLViewerShim::~QGLViewerShim() = default;

void QGLViewerShim::initializeGL() {
  initializeOpenGLFunctions();
  // set the clear color from stored backgroundColor_
  const float rf = backgroundColor_.redF();
  const float gf = backgroundColor_.greenF();
  const float bf = backgroundColor_.blueF();
  const float af = backgroundColor_.alphaF();
  g2o::opengl::clear_color(rf, gf, bf, af);
  // Basic default GL state similar to what g2o viewers expect
#ifndef GL_ES
  g2o::opengl::enable(GL_LINE_SMOOTH);
  g2o::opengl::enable(GL_BLEND);
  g2o::opengl::enable(GL_DEPTH_TEST);
  g2o::opengl::enable(GL_NORMALIZE);
  g2o::opengl::shade_model(GL_FLAT);
  g2o::opengl::blend_func(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
#endif
  init();
}

void QGLViewerShim::paintGL() {
  // clear buffers
  g2o::opengl::clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // set up camera view
  g2o::opengl::matrix_mode(GL_MODELVIEW);
  g2o::opengl::load_identity();
  {
    const Vec pos = camera_.position();
    const Vec tgt = camera_.target();
    g2o::opengl::glu_look_at(
        static_cast<double>(pos.x()), static_cast<double>(pos.y()),
        static_cast<double>(pos.z()), static_cast<double>(tgt.x()),
        static_cast<double>(tgt.y()), static_cast<double>(tgt.z()), 0.0, 1.0,
        0.0);
  }

  // draw scene
  draw();

  // optional coordinate axis at the world origin
  if (axisIsDrawn_) drawAxis();
}

void QGLViewerShim::resizeGL(int w, int h) {
  viewportW_ = w;
  viewportH_ = h;
  g2o::opengl::viewport(0, 0, w, h);
  g2o::opengl::matrix_mode(GL_PROJECTION);
  g2o::opengl::load_identity();
#ifndef GL_ES
  if (h == 0) h = 1;
  const double aspect = static_cast<double>(w) / static_cast<double>(h);
  // use a reasonable fov
  g2o::opengl::glu_perspective(kFovDegrees, aspect, 0.01, 10000.0);
#endif
  g2o::opengl::matrix_mode(GL_MODELVIEW);
}

void QGLViewerShim::draw() {
  // Default no-op. Derived viewers override this method.
}

void QGLViewerShim::drawAxis() {
  g2o::opengl::line_width(2.0F);
  g2o::opengl::begin_lines();
  // X - red
  g2o::opengl::color3f(1.0F, 0.0F, 0.0F);
  g2o::opengl::vertex3f(0.0F, 0.0F, 0.0F);
  g2o::opengl::vertex3f(1.0F, 0.0F, 0.0F);
  // Y - green
  g2o::opengl::color3f(0.0F, 1.0F, 0.0F);
  g2o::opengl::vertex3f(0.0F, 0.0F, 0.0F);
  g2o::opengl::vertex3f(0.0F, 1.0F, 0.0F);
  // Z - blue
  g2o::opengl::color3f(0.0F, 0.0F, 1.0F);
  g2o::opengl::vertex3f(0.0F, 0.0F, 0.0F);
  g2o::opengl::vertex3f(0.0F, 0.0F, 1.0F);
  g2o::opengl::end();
}

void QGLViewerShim::init() {
  // Default no-op. Derived viewers override this method.
}

void QGLViewerShim::setCamera(const Camera& cam) {
  camera_ = cam;
  update();
}

void QGLViewerShim::saveSnapshot(const QString& filename) {
  QImage img = grabFramebuffer();
  if (!img.save(filename)) {
    G2O_ERROR("Failed to save snapshot {}", filename.toStdString());
  }
}

void QGLViewerShim::saveSnapshot(const QString& filename,
                                 bool /*includeAlpha*/) {
  saveSnapshot(filename);
}

void QGLViewerShim::saveStateToFile(const QString& name) {
  if (name.isEmpty()) return;
#if !G2O_HAVE_JSON
  G2O_WARN("nlohmann/json not available - cannot save state to {}",
           name.toStdString());
  return;
#else
  try {
    nlohmann::json j;
    {
      const auto p = camera_.position();
      j["camera"]["position"] = {p.x(), p.y(), p.z()};
      const auto up = camera_.upVector();
      j["camera"]["up"] = {up.x(), up.y(), up.z()};
      const auto tgt = camera_.target();
      j["camera"]["target"] = {tgt.x(), tgt.y(), tgt.z()};
    }
    j["background"] = {backgroundColor_.red(), backgroundColor_.green(),
                       backgroundColor_.blue(), backgroundColor_.alpha()};
    j["axisIsDrawn"] = axisIsDrawn_;

    std::ofstream ofs(name.toStdString());
    if (!ofs) {
      qWarning() << "QGLViewerShim: failed to open state file for writing:"
                 << name;
      return;
    }
    ofs << j.dump(2) << '\n';
  } catch (const std::exception& e) {
    G2O_WARN("Exception while saving state: {}", e.what());
  }
#endif
}

void QGLViewerShim::restoreStateFromFile(const QString& name) {
  if (name.isEmpty()) return;
#if !G2O_HAVE_JSON
  G2O_WARN("nlohmann/json not available - cannot restore state from {}",
           name.toStdString());
  return;
#else
  try {
    std::ifstream ifs(name.toStdString());
    if (!ifs) {
      G2O_WARN("QGLViewerShim: failed to open state file for reading: {}",
               name.toStdString());
      return;
    }
    nlohmann::json j;
    ifs >> j;
    if (j.contains("camera") && j["camera"].contains("position")) {
      auto pos = j["camera"]["position"];
      if (pos.is_array() && pos.size() >= 3) {
        camera_.setPosition(
            Vec(pos[0].get<float>(), pos[1].get<float>(), pos[2].get<float>()));
      }
      if (j["camera"].contains("up")) {
        auto up = j["camera"]["up"];
        if (up.is_array() && up.size() >= 3) {
          camera_.setUpVector(
              Vec(up[0].get<float>(), up[1].get<float>(), up[2].get<float>()));
        }
      }
    }
    if (j.contains("camera") && j["camera"].contains("target")) {
      auto t = j["camera"]["target"];
      if (t.is_array() && t.size() >= 3) {
        camera_.setTarget(
            Vec(t[0].get<float>(), t[1].get<float>(), t[2].get<float>()));
      }
    }
    // No recomputation needed here; camera position is authoritative.
    if (j.contains("background") && j["background"].is_array() &&
        j["background"].size() >= 4) {
      auto b = j["background"];
      backgroundColor_.setRgb(b[0].get<int>(), b[1].get<int>(), b[2].get<int>(),
                              b[3].get<int>());
      // update GL clear color if context exists
      if (context()) {
        makeCurrent();
        g2o::opengl::clear_color(
            backgroundColor_.redF(), backgroundColor_.greenF(),
            backgroundColor_.blueF(), backgroundColor_.alphaF());
        doneCurrent();
      }
    }
    if (j.contains("axisIsDrawn")) axisIsDrawn_ = j["axisIsDrawn"].get<bool>();
    update();
  } catch (const std::exception& e) {
    qWarning() << "QGLViewerShim: exception while restoring state:" << e.what();
  }
#endif
}

// removed setStateFileName: state is provided per-call to save/restore

// slot implementations
void QGLViewerShim::setAxisIsDrawn(bool draw) {
  axisIsDrawn_ = draw;
  update();
}

void QGLViewerShim::setBackgroundColor(const QColor& c) {
  backgroundColor_ = c;
  QPalette p = palette();
  p.setColor(backgroundRole(), c);
  setPalette(p);
  // update GL clear color if we have a valid context
  if (context()) {
    makeCurrent();
    const float rf = backgroundColor_.redF();
    const float gf = backgroundColor_.greenF();
    const float bf = backgroundColor_.blueF();
    const float af = backgroundColor_.alphaF();
    g2o::opengl::clear_color(rf, gf, bf, af);
    doneCurrent();
  }
  update();
}

// --- Basic input handling -------------------------------------------------

void QGLViewerShim::mousePressEvent(QMouseEvent* event) {
  lastMousePos_ = event->pos();
  leftPressed_ = (event->buttons() & Qt::LeftButton) != 0U;
  middlePressed_ = (event->buttons() & Qt::MiddleButton) != 0U;
  rightPressed_ = (event->buttons() & Qt::RightButton) != 0U;
  event->accept();
}

void QGLViewerShim::mouseMoveEvent(QMouseEvent* event) {
  QPoint pos = event->pos();
  QPoint delta = pos - lastMousePos_;

  if (leftPressed_) {
    const Vec pos = camera_.position();
    const Vec tgt = camera_.target();
    const Vec v = pos - tgt;

    QVector3D vq(v.x(), v.y(), v.z());
    QVector3D upq(camera_.upVector().x(), camera_.upVector().y(),
                  camera_.upVector().z());

    QVector3D viewDirN = vq;
    if (!qFuzzyIsNull(viewDirN.length()))
      viewDirN.normalize();
    else
      viewDirN = QVector3D(0, 0, -1);

    QVector3D right = QVector3D::crossProduct(viewDirN, upq);
    if (!qFuzzyIsNull(right.length()))
      right.normalize();
    else
      right = QVector3D(1, 0, 0);

    QVector3D worldUp(0, 1, 0);

    // sensitivity: 0.5 degrees per pixel
    constexpr float kDegPerPixel = 0.5F;
    const float yawDeg = -delta.x() * kDegPerPixel;
    const float pitchDeg = delta.y() * kDegPerPixel;

    QQuaternion qPitch = QQuaternion::fromAxisAndAngle(right, pitchDeg);
    QQuaternion qYaw = QQuaternion::fromAxisAndAngle(worldUp, yawDeg);
    QQuaternion q = qYaw * qPitch;

    QVector3D rotatedVq = q.rotatedVector(vq);
    QVector3D rotatedUpq = q.rotatedVector(upq);

    Vec rotatedV(rotatedVq.x(), rotatedVq.y(), rotatedVq.z());
    Vec rotatedUp(rotatedUpq.x(), rotatedUpq.y(), rotatedUpq.z());

    camera_.setPosition(Vec(rotatedV.x() + tgt.x(), rotatedV.y() + tgt.y(),
                            rotatedV.z() + tgt.z()));
    camera_.setUpVector(rotatedUp);
  } else if (middlePressed_ || (leftPressed_ && rightPressed_)) {
    // translate target in view plane (approximate behavior from original)
    const float dist = camera_.distance();
    // Scale pan by world-units-per-pixel at the current distance. This makes
    // panning viewport-aware and much less sensitive than a fixed multiplier.
    const float unitsPerPixel = unitsPerPixelForDistance(dist, viewportH_);
    float tx = -delta.x() * unitsPerPixel;
    float ty = delta.y() * unitsPerPixel;
    // translate in camera view plane: compute right and up vectors
    Vec viewDir = camera_.target() - camera_.position();
    float vlen =
        std::sqrt((viewDir.x() * viewDir.x()) + (viewDir.y() * viewDir.y()) +
                  (viewDir.z() * viewDir.z()));
    Vec viewDirN = vlen > 1e-6F ? viewDir / vlen : Vec(0, 0, -1);
    Vec up = camera_.upVector();
    Vec right = QVector3D::crossProduct(viewDirN, up);
    float rlen = std::sqrt((right.x() * right.x()) + (right.y() * right.y()) +
                           (right.z() * right.z()));
    if (rlen > 1e-6F)
      right /= rlen;
    else
      right = Vec(1, 0, 0);
    Vec upCam = QVector3D::crossProduct(right, viewDirN);
    Vec shift = right * tx + upCam * ty;
    camera_.setTarget(camera_.target() + shift);
    camera_.setPosition(camera_.position() + shift);
  } else if (rightPressed_) {
    // dolly / zoom: change distance along the view vector by adjusting the
    float dist = camera_.distance();
    float newDist = std::clamp(dist + (delta.y() * 0.05F), 0.1F,
                               std::numeric_limits<float>::max());
    camera_.setDistance(newDist);
  }

  lastMousePos_ = pos;
  update();
  event->accept();
}

void QGLViewerShim::mouseReleaseEvent(QMouseEvent* event) {
  Q_UNUSED(event)
  leftPressed_ = middlePressed_ = rightPressed_ = false;
}

void QGLViewerShim::wheelEvent(QWheelEvent* event) {
  // zoom in/out
  const int delta = event->angleDelta().y();
  // update camera distance from target (scale the offset vector)
  const float scale = (delta > 0) ? 0.9F : 1.1F;
  float dist = camera_.distance();
  float newDist =
      std::clamp(dist * scale, 0.01F, std::numeric_limits<float>::max());
  camera_.setDistance(newDist);
  update();
  event->accept();
}

}  // namespace g2o::viewer
