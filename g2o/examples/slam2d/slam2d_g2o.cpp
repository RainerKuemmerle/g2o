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

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "main_window.h"
using namespace std;
using namespace g2o;

G2O_USE_TYPE_GROUP(slam2d);

int main(int argc, char** argv) {
  QApplication qapp(argc, argv);

  MainWindow mw;
  mw.show();

  mw.viewer->graph = new SparseOptimizer();

  typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
  typedef LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // Gauss Newton
  auto linearSolverGN = g2o::make_unique<SlamLinearSolver>();
  linearSolverGN->setBlockOrdering(false);
  mw.solverGaussNewton = new OptimizationAlgorithmGaussNewton(
      g2o::make_unique<SlamBlockSolver>(std::move(linearSolverGN)));

  // Levenberg
  auto linearSolverLM = g2o::make_unique<SlamLinearSolver>();
  linearSolverLM->setBlockOrdering(false);
  mw.solverLevenberg = new OptimizationAlgorithmLevenberg(
      g2o::make_unique<SlamBlockSolver>(std::move(linearSolverLM)));

  mw.viewer->graph->setAlgorithm(mw.solverGaussNewton);
  return qapp.exec();
}
