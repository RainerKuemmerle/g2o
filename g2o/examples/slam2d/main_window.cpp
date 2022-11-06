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

#include "main_window.h"
//#include "moc_main_window.cpp"

#include <QFileDialog>
#include <fstream>
#include <iostream>

#include "g2o/core/block_solver.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >;
using SlamLinearSolver =
    g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) { setupUi(this); }

void MainWindow::on_actionLoad_triggered(bool) {
  viewer->graph->clear();
  const QString filename = QFileDialog::getOpenFileName(
      this, "Load g2o file", "", "g2o files (*.g2o);;All Files (*)");
  if (!filename.isNull()) {
    std::ifstream ifs(filename.toStdString().c_str());
    viewer->graph->load(ifs);
    std::cerr << "Graph loaded with " << viewer->graph->vertices().size()
              << " vertices and " << viewer->graph->edges().size()
              << " Measurements" << std::endl;
  }
  viewer->update();
  fixGraph();
}

void MainWindow::on_actionSave_triggered(bool) {
  const QString filename = QFileDialog::getSaveFileName(this, "Save g2o file", "",
                                                  "g2o files (*.g2o)");
  if (!filename.isNull()) {
    std::ofstream fout(filename.toStdString().c_str());
    viewer->graph->save(fout);
    if (fout.good())
      std::cerr << "Saved " << filename.toStdString() << std::endl;
    else
      std::cerr << "Error while saving file" << std::endl;
  }
}

void MainWindow::on_actionQuit_triggered(bool) { close(); }

void MainWindow::on_btnOptimize_clicked() {
  if (viewer->graph->vertices().empty() || viewer->graph->edges().empty()) {
    std::cerr << "Graph has no vertices / edges" << std::endl;
    return;
  }

  viewer->graph->initializeOptimization();

  if (rbLevenberg->isChecked()) {
    if (dynamic_cast<g2o::OptimizationAlgorithmLevenberg*>(
            viewer->graph->solver().get()) == nullptr)
      viewer->graph->setAlgorithm(createLevenberg());
  } else {
    if (dynamic_cast<g2o::OptimizationAlgorithmGaussNewton*>(
            viewer->graph->solver().get()) == nullptr)
      viewer->graph->setAlgorithm(createGaussNewton());
  }

  const int maxIterations = spIterations->value();
  const int iter = viewer->graph->optimize(maxIterations);
  if (maxIterations > 0 && !iter) {
    std::cerr << "Optimization failed, result might be invalid" << std::endl;
  }

  if (cbCovariances->isChecked()) {
    // TODO(Rainer): implementation of covariance estimates
    // viewer->graph->solver()->computeMarginals();
  }
  viewer->drawCovariance = cbCovariances->isChecked();

  viewer->update();
}

void MainWindow::on_btnInitialGuess_clicked() {
  viewer->graph->computeInitialGuess();
  viewer->drawCovariance = false;
  viewer->update();
}

void MainWindow::fixGraph() {
  if (viewer->graph->vertices().empty() || viewer->graph->edges().empty()) {
    return;
  }

  // check for vertices to fix to remove DoF
  const bool gaugeFreedom = viewer->graph->gaugeFreedom();
  auto gauge = viewer->graph->findGauge();
  if (gaugeFreedom) {
    if (!gauge) {
      std::cerr << "cannot find a vertex to fix in this thing" << std::endl;
      return;
    }
    std::cerr << "graph is fixed by node " << gauge->id() << std::endl;
    gauge->setFixed(true);
  } else {
    std::cerr << "graph is fixed by priors" << std::endl;
  }

  viewer->graph->setVerbose(true);
  viewer->graph->computeActiveErrors();
}

std::unique_ptr<g2o::OptimizationAlgorithm> MainWindow::createGaussNewton() {
  auto linearSolverGN = g2o::make_unique<SlamLinearSolver>();
  linearSolverGN->setBlockOrdering(false);
  return std::unique_ptr<g2o::OptimizationAlgorithm>(
      new g2o::OptimizationAlgorithmGaussNewton(
          g2o::make_unique<SlamBlockSolver>(std::move(linearSolverGN))));
}

std::unique_ptr<g2o::OptimizationAlgorithm> MainWindow::createLevenberg() {
  // Levenberg
  auto linearSolverLM = g2o::make_unique<SlamLinearSolver>();
  linearSolverLM->setBlockOrdering(false);
  return std::unique_ptr<g2o::OptimizationAlgorithm>(
      new g2o::OptimizationAlgorithmLevenberg(
          g2o::make_unique<SlamBlockSolver>(std::move(linearSolverLM))));
}
