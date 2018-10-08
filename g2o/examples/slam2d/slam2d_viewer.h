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

#ifndef SLAM2D_VIEWER_H
#define SLAM2D_VIEWER_H

#include "qglviewer.h"

namespace g2o {

  class SparseOptimizer;

  class Slam2DViewer : public QGLViewer
  {
    public:
      Slam2DViewer(QWidget* parent=NULL, const QGLWidget* shareWidget=0, Qt::WindowFlags flags=0);
      ~Slam2DViewer();
      virtual void draw();
      void init();

    public:
      SparseOptimizer* graph;
      bool drawCovariance;
  };

} // end namespace

#endif
