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

#include <QFileDialog>
#include <fstream>
#include <iostream>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
using namespace std;

namespace {
using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >;
using SlamLinearSolver =
    g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

// Gauss Newton
g2o::OptimizationAlgorithm* createGauss() {
  auto linearSolverGN = std::make_unique<SlamLinearSolver>();
  linearSolverGN->setBlockOrdering(false);
  return new g2o::OptimizationAlgorithmGaussNewton(
      std::make_unique<SlamBlockSolver>(std::move(linearSolverGN)));
}

// Levenberg
g2o::OptimizationAlgorithm* createLevenberg() {
  auto linearSolverLM = std::make_unique<SlamLinearSolver>();
  linearSolverLM->setBlockOrdering(false);
  return new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<SlamBlockSolver>(std::move(linearSolverLM)));
}

}  // namespace

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) { setupUi(this); }

MainWindow::~MainWindow() {}

void MainWindow::on_actionLoad_triggered(bool) {
  viewer->graph->clear();
  QString filename = QFileDialog::getOpenFileName(
      this, "Load g2o file", "", "g2o files (*.g2o);;All Files (*)");
  if (!filename.isNull()) {
    ifstream ifs(filename.toStdString().c_str());
    viewer->graph->load(ifs);
    cerr << "Graph loaded with " << viewer->graph->vertices().size()
         << " vertices and " << viewer->graph->edges().size() << " Measurements"
         << endl;
  }
  viewer->update();
  fixGraph();
}

void MainWindow::on_actionSave_triggered(bool) {
  QString filename = QFileDialog::getSaveFileName(this, "Save g2o file", "",
                                                  "g2o files (*.g2o)");
  if (!filename.isNull()) {
    ofstream fout(filename.toStdString().c_str());
    viewer->graph->save(fout);
    if (fout.good())
      cerr << "Saved " << filename.toStdString() << endl;
    else
      cerr << "Error while saving file" << endl;
  }
}

void MainWindow::on_actionQuit_triggered(bool) { close(); }

void MainWindow::on_btnOptimize_clicked() {
  if (viewer->graph->vertices().size() == 0 ||
      viewer->graph->edges().size() == 0) {
    cerr << "Graph has no vertices / edges" << endl;
    return;
  }

  viewer->graph->initializeOptimization();

  if (rbGauss->isChecked())
    viewer->graph->setAlgorithm(createGauss());
  else if (rbLevenberg->isChecked())
    viewer->graph->setAlgorithm(createLevenberg());
  else
    viewer->graph->setAlgorithm(createGauss());

  int maxIterations = spIterations->value();
  int iter = viewer->graph->optimize(maxIterations);
  if (maxIterations > 0 && !iter) {
    cerr << "Optimization failed, result might be invalid" << endl;
  }

  if (cbCovariances->isChecked()) {
    std::vector<std::pair<int, int> > cov_vertices;
    for (const auto& vertex_index : viewer->graph->vertices()) {
      auto* vertex =
          static_cast<g2o::OptimizableGraph::Vertex*>(vertex_index.second);
      if (!vertex->fixed())
        cov_vertices.emplace_back(vertex->hessianIndex(),
                                  vertex->hessianIndex());
    }
    viewer->covariances.clear(true);
    std::cerr << "Compute covariance matrices" << std::endl;
    bool cov_result =
        viewer->graph->computeMarginals(viewer->covariances, cov_vertices);
    viewer->drawCovariance = cov_result;
    std::cerr << (cov_result ? "Done." : "Failed") << std::endl;
  } else {
    viewer->drawCovariance = false;
  }
  viewer->update();
}

void MainWindow::on_btnInitialGuess_clicked() {
  viewer->graph->computeInitialGuess();
  viewer->drawCovariance = false;
  viewer->update();
}

void MainWindow::fixGraph() {
  if (viewer->graph->vertices().size() == 0 ||
      viewer->graph->edges().size() == 0) {
    return;
  }

  // check for vertices to fix to remove DoF
  bool gaugeFreedom = viewer->graph->gaugeFreedom();
  g2o::OptimizableGraph::Vertex* gauge = viewer->graph->findGauge();
  if (gaugeFreedom) {
    if (!gauge) {
      cerr << "cannot find a vertex to fix in this thing" << endl;
      return;
    } else {
      cerr << "graph is fixed by node " << gauge->id() << endl;
      gauge->setFixed(true);
    }
  } else {
    cerr << "graph is fixed by priors" << endl;
  }

  viewer->graph->setVerbose(true);
  viewer->graph->computeActiveErrors();
}
