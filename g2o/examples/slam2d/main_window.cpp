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

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/estimate_propagator.h"

#include <QFileDialog>

#include <fstream>
#include <iostream>
using namespace std;

MainWindow::MainWindow(QWidget * parent, Qt::WindowFlags flags) :
  QMainWindow(parent, flags)
{
  setupUi(this);
}

MainWindow::~MainWindow()
{
}

void MainWindow::on_actionLoad_triggered(bool)
{
  viewer->graph->clear();
  QString filename = QFileDialog::getOpenFileName(this, "Load g2o file", "", "g2o files (*.g2o);;All Files (*)");
  if (! filename.isNull()) {
    ifstream ifs(filename.toStdString().c_str());
    viewer->graph->load(ifs);
    cerr << "Graph loaded with " << viewer->graph->vertices().size() << " vertices and "
      << viewer->graph->edges().size() << " measurments" << endl;
  }
  viewer->updateGL();
  fixGraph();
}

void MainWindow::on_actionSave_triggered(bool)
{
  QString filename = QFileDialog::getSaveFileName(this, "Save g2o file", "", "g2o files (*.g2o)");
  if (! filename.isNull()) {
    ofstream fout(filename.toStdString().c_str());
    viewer->graph->save(fout);
    if (fout.good())
      cerr << "Saved " << filename.toStdString() << endl;
    else
      cerr << "Error while saving file" << endl;
  }
}

void MainWindow::on_actionQuit_triggered(bool)
{
  close();
}

void MainWindow::on_btnOptimize_clicked()
{
  if (viewer->graph->vertices().size() == 0 || viewer->graph->edges().size() == 0) {
    cerr << "Graph has no vertices / egdes" << endl;
    return;
  }

  viewer->graph->initializeOptimization();

  if (rbGauss->isChecked())
    viewer->graph->setAlgorithm(solverGaussNewton);
  else if (rbLevenberg->isChecked())
    viewer->graph->setAlgorithm(solverLevenberg);
  else
    viewer->graph->setAlgorithm(solverGaussNewton);

  int maxIterations = spIterations->value();
  int iter = viewer->graph->optimize(maxIterations);
  if (maxIterations > 0 && !iter){
    cerr << "Optimization failed, result might be invalid" << endl;
  }

  if (cbCovariances->isChecked()) {
    // TODO
    //viewer->graph->solver()->computeMarginals();
  }
  viewer->drawCovariance = cbCovariances->isChecked();

  viewer->updateGL();
}

void MainWindow::on_btnInitialGuess_clicked()
{
  viewer->graph->computeInitialGuess();
  viewer->drawCovariance = false;
  viewer->updateGL();
}

void MainWindow::fixGraph()
{
  if (viewer->graph->vertices().size() == 0 || viewer->graph->edges().size() == 0) {
    return;
  }

  // check for vertices to fix to remove DoF
  bool gaugeFreedom = viewer->graph->gaugeFreedom();
  g2o::OptimizableGraph::Vertex* gauge = viewer->graph->findGauge();
  if (gaugeFreedom) {
    if (! gauge) {
      cerr <<  "cannot find a vertex to fix in this thing" << endl;
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
