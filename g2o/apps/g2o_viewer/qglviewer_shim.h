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

// Minimal compatibility shim to replace QGLViewer with QOpenGLWidget
#ifndef G2O_QGLVIEWER_SHIM_H
#define G2O_QGLVIEWER_SHIM_H

#include <QColor>
#include <QImage>
#include <QOpenGLFunctions>
#include <QOpenGLWidget>
#include <QPoint>
#include <QString>
#include <QSurfaceFormat>
#include <QVector3D>
#include <Qt>

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;

namespace g2o::viewer {

using Vec = QVector3D;

class Camera {
 public:
  Camera() = default;
  virtual ~Camera() = default;
  virtual void setPosition(const Vec& p) { pos_ = p; }
  [[nodiscard]] virtual Vec position() const { return pos_; }
  virtual void setZNear(float v) { zNear_ = v; }
  virtual void setZFar(float v) { zFar_ = v; }
  [[nodiscard]] virtual float zNear() const { return zNear_; }
  [[nodiscard]] virtual float zFar() const { return zFar_; }
  virtual void setUpVector(const Vec& v) { up_ = v; }
  [[nodiscard]] virtual Vec upVector() const { return up_; }
  virtual void setTarget(const Vec& t) { target_ = t; }
  [[nodiscard]] virtual Vec target() const { return target_; }
  [[nodiscard]] virtual float distance() const {
    float dist = (pos_ - target_).length();
    if (dist < 1e-6F) dist = 1.0F;
    return dist;
  }
  virtual void setDistance(float newDist) {
    const Vec tgt = target_;
    Vec v = pos_ - tgt;
    float dist = distance();
    if (dist > 1e-6F)
      v *= (newDist / dist);
    else
      v = Vec(0, 0, -newDist);
    pos_ = tgt + v;
  }
  virtual void lookAt(const Vec& /*v*/) {}

 private:
  Vec pos_;
  Vec up_;
  Vec target_ = Vec(0, 0, 0);
  float zNear_ = 0.1F;
  float zFar_ = 1000.0F;
};

class QGLViewerShim : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

 public:
  explicit QGLViewerShim(QWidget* parent = nullptr);
  ~QGLViewerShim() override;

  // QGLViewer-like API used by g2o viewers
  virtual void draw();
  virtual void init();

  [[nodiscard]] const Camera& camera() const { return camera_; }
  void setCamera(const Camera& cam);

  void saveSnapshot(const QString& filename);
  void saveSnapshot(const QString& filename, bool /*includeAlpha*/);
  void setSnapshotFormat(const QString& /*fmt*/) {}
  void setSnapshotQuality(int /*q*/) {}
  // legacy QGLViewer convenience methods (stubs)

 public Q_SLOTS:  // NOLINT
  void setAxisIsDrawn(bool draw);
  void setBackgroundColor(const QColor& c);
  void saveStateToFile(const QString& name);
  void restoreStateFromFile(const QString& name);

 protected:
  // QOpenGLWidget overrides
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int w, int h) override;

  // draw world axes at origin when enabled
  void drawAxis();

  // Input events to provide basic camera interaction (rotate/translate/zoom)
  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  Camera camera_;
  // State file path is passed per-call to save/restore methods.
  // Simple orbit-style camera state
  QPoint lastMousePos_;
  bool leftPressed_ = false;
  bool middlePressed_ = false;
  bool rightPressed_ = false;
  int viewportW_ = 640;
  int viewportH_ = 480;
  QColor backgroundColor_ = Qt::black;
  bool axisIsDrawn_ = false;
};

}  // namespace g2o::viewer

#endif  // G2O_QGLVIEWER_SHIM_H
