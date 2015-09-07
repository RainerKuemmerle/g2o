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

#include "qglviewer.h"
#include "g2o_viewer_api.h"
#include "g2o/core/hyper_graph_action.h"

namespace g2o {

  class SparseOptimizer;

  /**
   * \brief OpenGL based viewer for the graph
   */
  class G2O_VIEWER_API G2oQGLViewer : public QGLViewer
  {
    public:
      G2oQGLViewer(QWidget* parent=NULL, const QGLWidget* shareWidget=0, Qt::WindowFlags flags=0);
      ~G2oQGLViewer();
      void draw();
      void init();

      /**
       * the viewer uses a display list to cache the drawing, use setUpdateDisplay() to force
       * the creation of an updated display list.
       */
      bool updateDisplay() const { return _updateDisplay;}
      void setUpdateDisplay(bool updateDisplay);

      DrawAction::Parameters* parameters() { return _drawActionParameters;}

    public:
      SparseOptimizer* graph;

    protected:
      HyperGraphElementAction* _drawActions;
      GLuint _drawList;
      bool _updateDisplay;
      DrawAction::Parameters* _drawActionParameters;
  };

} // end namespace

#endif
