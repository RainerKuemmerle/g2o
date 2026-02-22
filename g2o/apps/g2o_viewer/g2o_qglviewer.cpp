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
#include "g2o/stuff/logger.h"
#include "g2o/stuff/opengl_wrapper.h"

namespace g2o {

namespace {

void applyAction(HyperGraph& graph, HyperGraphElementAction& action,
                 HyperGraphElementAction::Parameters& parameters) {
  for (auto& it : graph.vertices()) {
    (action)(*it.second, parameters);
  }
  for (const auto& it : graph.edges()) {
    (action)(*it, parameters);
  }
}

}  // end anonymous namespace

G2oQGLViewer::G2oQGLViewer(QWidget* parent)
    : viewer::QGLViewerShim(parent), drawActions_(nullptr) {
  setAxisIsDrawn(false);
}

G2oQGLViewer::~G2oQGLViewer() {
  // Ensure we have a current GL context before deleting GL resources
  makeCurrent();
  if (drawList_.has_value()) {
    glDeleteLists(drawList_.value(), 1);
    drawList_ = std::nullopt;
  }
  doneCurrent();
}

void G2oQGLViewer::draw() {
  if (!graph) return;

  if (drawActions_ == nullptr) {
    drawActions_ = HyperGraphActionLibrary::instance()->actionByName("draw");
  }

  if (!drawActions_) {  // avoid segmentation fault in release build
    static bool warned = false;
    if (!warned) {
      G2O_ERROR(
          "G2oQGLViewer: no draw action found in HyperGraphActionLibrary");
      warned = true;
    }
    return;
  }

  if (!drawList_.has_value()) {
    drawList_ = glGenLists(1);
    assert(drawList_ != 0);
  }

  if (updateDisplay_) {
    updateDisplay_ = false;
    glNewList(drawList_.value(), GL_COMPILE_AND_EXECUTE);
    SparseOptimizer& optimizer = *graph;
    applyAction(optimizer, *drawActions_, drawActionParameters_);
    glEndList();
  } else {
    glCallList(drawList_.value());
  }
}

void G2oQGLViewer::init() {
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

  setAxisIsDrawn(true);
  // don't save state (no-op under new API: state passed per-call)
}

void G2oQGLViewer::setUpdateDisplay(bool updateDisplay) {
  updateDisplay_ = updateDisplay;
}

}  // namespace g2o
