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

#ifndef G2O_QGL_GRAPH_VIEWER_H
#define G2O_QGL_GRAPH_VIEWER_H

#include "g2o/core/hyper_graph_action.h"
#include "g2o_viewer_api.h"
#include "qglviewer.h"

namespace g2o {

class SparseOptimizer;

/**
 * \brief OpenGL based viewer for the graph
 */
class G2O_VIEWER_API G2oQGLViewer : public QGLViewer {
 public:
  explicit G2oQGLViewer(QWidget* parent = nullptr,
                        const QGLWidget* shareWidget = nullptr);
  G2oQGLViewer(const G2oQGLViewer&) = delete;
  G2oQGLViewer& operator=(const G2oQGLViewer&) = delete;

  ~G2oQGLViewer() override;
  void draw() override;
  void init() override;

  /**
   * the viewer uses a display list to cache the drawing, use setUpdateDisplay()
   * to force the creation of an updated display list.
   */
  bool updateDisplay() const { return updateDisplay_; }
  void setUpdateDisplay(bool updateDisplay);

  std::shared_ptr<DrawAction::Parameters> parameters() const {
    return drawActionParameters_;
  }

  SparseOptimizer* graph = nullptr;

 protected:
  HyperGraphElementAction::HyperGraphElementActionPtr drawActions_;
  GLuint drawList_ = 0;
  bool updateDisplay_ = true;
  std::shared_ptr<DrawAction::Parameters> drawActionParameters_;
};

}  // namespace g2o

#endif
