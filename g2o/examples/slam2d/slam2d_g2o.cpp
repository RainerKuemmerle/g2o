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

#include <QApplication>
#include <iostream>
#include <memory>

#include "g2o/core/factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "main_window.h"

G2O_USE_TYPE_GROUP(slam2d);

int main(int argc, char** argv) {
  const QApplication qapp(argc, argv);

  MainWindow mw;
  mw.viewer->graph = std::make_unique<g2o::SparseOptimizer>();
  mw.viewer->graph->setAlgorithm(MainWindow::createGaussNewton());
  mw.show();
  return QApplication::exec();
}
