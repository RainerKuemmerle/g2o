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

#include <QComboBox>
#include <QDoubleValidator>
#include <QFileDialog>
#include <QStandardItemModel>
#include <cassert>
#include <fstream>
#include <iostream>

#include "g2o/core/estimate_propagator.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "viewer_properties_widget.h"

using std::cerr;
using std::endl;
using std::string;

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)

{
  setupUi(this);
  leKernelWidth->setValidator(
      new QDoubleValidator(-std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max(), 7, this));
  plainTextEdit->setMaximumBlockCount(1000);
  btnForceStop->hide();
  QObject::connect(cbDrawAxis, SIGNAL(toggled(bool)), viewer,
                   SLOT(setAxisIsDrawn(bool)));
}

void MainWindow::on_actionLoad_triggered(bool) {
  const QString filename = QFileDialog::getOpenFileName(
      this, "Load g2o file", "", "g2o files (*.g2o);;All Files (*)");
  if (!filename.isNull()) {
    loadFromFile(filename);
  }
}

void MainWindow::on_actionSave_triggered(bool) {
  const QString filename = QFileDialog::getSaveFileName(
      this, "Save g2o file", "", "g2o files (*.g2o)");
  if (!filename.isNull()) {
    std::ofstream fout(filename.toStdString().c_str());
    viewer->graph->save(fout);
    if (fout.good())
      cerr << "Saved " << filename.toStdString() << endl;
    else
      cerr << "Error while saving file" << endl;
  }
}

void MainWindow::on_btnOptimize_clicked() {
  if (viewer->graph->vertices().empty() || viewer->graph->edges().empty()) {
    cerr << "Graph has no vertices / edges" << endl;
    return;
  }

  bool allocatedNewSolver;
  const bool allocateStatus = allocateSolver(allocatedNewSolver);
  if (!allocateStatus) {
    cerr << "Error while allocating solver" << endl;
    return;
  }
  if (allocatedNewSolver) prepare();
  setRobustKernel();

  btnOptimize->hide();
  btnForceStop->show();

  forceStopFlag_ = false;
  viewer->graph->setForceStopFlag(&forceStopFlag_);

  const int maxIterations = spIterations->value();
  const int iter = viewer->graph->optimize(maxIterations);
  if (maxIterations > 0 && !iter) {
    cerr << "Optimization failed, result might be invalid" << endl;
  }

  btnOptimize->show();
  btnForceStop->hide();

  viewer->setUpdateDisplay(true);
  viewer->update();
  forceStopFlag_ = false;
}

void MainWindow::on_btnInitialGuess_clicked() {
  if (viewer->graph->activeEdges().empty())
    viewer->graph->initializeOptimization();

  switch (cbxIniitialGuessMethod->currentIndex()) {
    case 0:
      // spanning tree
      viewer->graph->computeInitialGuess();
      break;
    case 1:
      // odometry
      {
        g2o::EstimatePropagatorCostOdometry costFunction(viewer->graph);
        viewer->graph->computeInitialGuess(costFunction);
      }
      break;
    default:
      cerr << __PRETTY_FUNCTION__ << " Unknown initialization method" << endl;
      break;
  }

  viewer->setUpdateDisplay(true);
  viewer->update();
}

void MainWindow::on_btnSetZero_clicked() {
  if (viewer->graph->activeEdges().empty())
    viewer->graph->initializeOptimization();

  viewer->graph->setToOrigin();
  viewer->setUpdateDisplay(true);
  viewer->update();
}

void MainWindow::on_btnReload_clicked() {
  if (filename_.length() > 0) {
    cerr << "reloading " << filename_ << endl;
    viewer->graph->clear();
    viewer->graph->load(filename_.c_str());
    viewer->setUpdateDisplay(true);
    viewer->update();
  }
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
      cerr << "cannot find a vertex to fix in this thing" << endl;
      return;
    }
    cerr << "graph is fixed by node " << gauge->id() << endl;
    gauge->setFixed(true);

  } else {
    cerr << "graph is fixed by priors or nodes are already fixed" << endl;
  }

  viewer->graph->setVerbose(true);
  // viewer->graph->computeActiveErrors();
}

void MainWindow::on_actionQuit_triggered(bool) { close(); }

void MainWindow::updateDisplayedSolvers() {
  coOptimizer->clear();
  knownSolvers_.clear();
  const g2o::OptimizationAlgorithmFactory::CreatorList& knownSolvers =
      g2o::OptimizationAlgorithmFactory::instance()->creatorList();

  bool varFound = false;
  string varType;
  for (const auto& knownSolver : knownSolvers) {
    const g2o::OptimizationAlgorithmProperty& sp = knownSolver->property();
    if (sp.name == "gn_var" || sp.name == "gn_var_cholmod") {
      varType = sp.type;
      varFound = true;
      break;
    }
  }

  if (varFound) {
    for (const auto& knownSolver : knownSolvers) {
      const g2o::OptimizationAlgorithmProperty& sp = knownSolver->property();
      if (sp.type == varType) {
        coOptimizer->addItem(QString::fromStdString(sp.name));
        knownSolvers_.push_back(sp);
      }
    }
  }

  std::map<string, std::vector<g2o::OptimizationAlgorithmProperty> >
      solverLookUp;

  for (const auto& knownSolver : knownSolvers) {
    const g2o::OptimizationAlgorithmProperty& sp = knownSolver->property();
    if (varFound && varType == sp.type) continue;
    solverLookUp[sp.type].push_back(sp);
  }

  for (auto& it : solverLookUp) {
    if (!knownSolvers_.empty()) {
      coOptimizer->insertSeparator(coOptimizer->count());
      knownSolvers_.emplace_back();
    }
    const std::vector<g2o::OptimizationAlgorithmProperty>& vsp = it.second;
    for (const auto& j : vsp) {
      coOptimizer->addItem(QString::fromStdString(j.name));
      knownSolvers_.push_back(j);
    }
  }
}

bool MainWindow::load(const QString& filename) {
  viewer->graph->clear();
  bool loadStatus = false;
  if (filename == "-") {
    cerr << "reading stdin" << endl;
    loadStatus = viewer->graph->load(std::cin);
  } else {
    std::ifstream ifs(filename.toStdString().c_str());
    if (!ifs) return false;
    loadStatus = viewer->graph->load(ifs);
  }
  if (!loadStatus) return false;
  lastSolver_ = -1;
  viewer->setUpdateDisplay(true);
  g2o::SparseOptimizer* optimizer = viewer->graph;

  // update the solvers which are suitable for this graph
  const std::set<int> vertDims = optimizer->dimensions();
  for (size_t i = 0; i < knownSolvers_.size(); ++i) {
    const g2o::OptimizationAlgorithmProperty& sp = knownSolvers_[i];
    if (sp.name.empty() && sp.desc.empty()) continue;

    const bool suitableSolver = optimizer->isSolverSuitable(sp, vertDims);
    qobject_cast<QStandardItemModel*>(coOptimizer->model())
        ->item(i)
        ->setEnabled(suitableSolver);
  }
  return loadStatus;
}

bool MainWindow::allocateSolver(bool& allocatedNewSolver) {
  if (coOptimizer->count() == 0) {
    cerr << "No solvers available" << endl;
    return false;
  }
  const int currentIndex = coOptimizer->currentIndex();
  const bool enabled = qobject_cast<QStandardItemModel*>(coOptimizer->model())
                           ->item(currentIndex)
                           ->isEnabled();

  if (!enabled) {
    cerr << "selected solver is not enabled" << endl;
    return false;
  }

  if (currentIndex == lastSolver_) return true;

  allocatedNewSolver = true;
  const QString strSolver = coOptimizer->currentText();

  // create the new algorithm
  g2o::OptimizationAlgorithmFactory* solverFactory =
      g2o::OptimizationAlgorithmFactory::instance();
  viewer->graph->setAlgorithm(solverFactory->construct(
      strSolver.toStdString(), currentOptimizationAlgorithmProperty_));

  lastSolver_ = currentIndex;
  return true;
}

bool MainWindow::prepare() {
  g2o::SparseOptimizer* optimizer = viewer->graph;
  if (currentOptimizationAlgorithmProperty_.requiresMarginalize) {
    cerr << "Marginalizing Landmarks" << endl;
    for (const auto& it : optimizer->vertices()) {
      auto* v = static_cast<g2o::OptimizableGraph::Vertex*>(it.second.get());
      const int vdim = v->dimension();
      v->setMarginalized(
          (vdim == currentOptimizationAlgorithmProperty_.landmarkDim));
    }
  } else {
    cerr << "Preparing (no marginalization of Landmarks)" << endl;
    for (const auto& it : optimizer->vertices()) {
      auto* v = static_cast<g2o::OptimizableGraph::Vertex*>(it.second.get());
      v->setMarginalized(false);
    }
  }
  viewer->graph->initializeOptimization();
  return true;
}

void MainWindow::setRobustKernel() {
  g2o::SparseOptimizer* optimizer = viewer->graph;
  const bool robustKernel = cbRobustKernel->isChecked();
  const double huberWidth = leKernelWidth->text().toDouble();
  // odometry edges are those whose node ids differ by 1
  const bool onlyLoop = cbOnlyLoop->isChecked();

  if (robustKernel) {
    const QString strRobustKernel = coRobustKernel->currentText();
    const g2o::AbstractRobustKernelCreator::Ptr creator =
        g2o::RobustKernelFactory::instance()->creator(
            strRobustKernel.toStdString());
    if (!creator) {
      cerr << strRobustKernel.toStdString() << " is not a valid robust kernel"
           << endl;
      return;
    }
    const g2o::RobustKernelPtr robustKernel = creator->construct();
    robustKernel->setDelta(huberWidth);
    for (const auto& it : optimizer->edges()) {
      auto* e = static_cast<g2o::OptimizableGraph::Edge*>(it.get());
      if (onlyLoop) {
        if (e->vertices().size() >= 2 &&
            std::abs(e->vertex(0)->id() - e->vertex(1)->id()) != 1) {
          e->setRobustKernel(robustKernel);
        }
      } else {
        e->setRobustKernel(robustKernel);
      }
    }
  } else {
    const g2o::RobustKernelPtr emptyKernel;
    for (const auto& it : optimizer->edges()) {
      auto* e = static_cast<g2o::OptimizableGraph::Edge*>(it.get());
      e->setRobustKernel(emptyKernel);
    }
  }
}

void MainWindow::on_btnForceStop_clicked() { forceStopFlag_ = true; }

bool MainWindow::loadFromFile(const QString& filename) {
  viewer->graph->clear();
  bool loadStatus = load(filename);
  if (loadStatus) {
    filename_ = filename.toStdString();
  }
  cerr << "loaded " << filename.toStdString() << " with "
       << viewer->graph->vertices().size() << " vertices and "
       << viewer->graph->edges().size() << " measurements" << endl;
  viewer->update();
  fixGraph();
  return loadStatus;
}

void MainWindow::on_actionWhite_Background_triggered(bool) {
  viewer->setBackgroundColor(QColor::fromRgb(255, 255, 255));
  viewer->update();
}

void MainWindow::on_actionDefault_Background_triggered(bool) {
  viewer->setBackgroundColor(QColor::fromRgb(51, 51, 51));
  viewer->update();
}

void MainWindow::on_actionProperties_triggered(bool) {
  if (!viewerPropertiesWidget_) {
    viewerPropertiesWidget_ = new ViewerPropertiesWidget(this);
    viewerPropertiesWidget_->setWindowTitle(tr("Drawing Options"));
  }
  viewerPropertiesWidget_->setViewer(viewer);
  viewerPropertiesWidget_->show();
}

void MainWindow::on_btnOptimizerParameters_clicked() {
  if (!optimizerPropertiesWidget_) {
    optimizerPropertiesWidget_ = new PropertiesWidget(this);
    optimizerPropertiesWidget_->setWindowTitle(
        tr("Internal Solver Properties"));
  }
  bool allocatedNewSolver;
  bool allocateStatus = allocateSolver(allocatedNewSolver);
  if (!allocateStatus) {
    cerr << "Error while allocating solver" << endl;
    return;
  }
  if (allocatedNewSolver) prepare();
  if (viewer->graph->solver()) {
    optimizerPropertiesWidget_->setProperties(
        const_cast<g2o::PropertyMap*>(&viewer->graph->solver()->properties()));
  } else {
    optimizerPropertiesWidget_->setProperties(nullptr);
  }
  optimizerPropertiesWidget_->show();
}

void MainWindow::on_actionSave_Screenshot_triggered(bool) {
  QString selectedFilter;
  QString filename = QFileDialog::getSaveFileName(
      this, "Save screen to a file", "viewer.png",
      "PNG files (*.png);;JPG files (*.jpg);;EPS files (*.eps)",
      &selectedFilter);

  if (!filename.isNull()) {
    // extract the file format from the filter options
    int spacePos = selectedFilter.indexOf(' ');
    assert(spacePos > 0 && "extracting the image format failed");
    QString format = selectedFilter.left(spacePos);
    // setting up the snapshot and save to file
    viewer->setSnapshotQuality(format == "JPG" ? 90 : -1);
    viewer->setSnapshotFormat(format);
    viewer->saveSnapshot(filename);
    cerr << "saved snapshot " << filename.toStdString() << "("
         << format.toStdString() << ")" << endl;
  }
}

void MainWindow::on_actionLoad_Viewer_State_triggered(bool) {
  QString filename = QFileDialog::getOpenFileName(
      this, "Load State", "camera.xml", "Camera/State file (*.xml)");
  if (!filename.isEmpty()) {
    viewer->setStateFileName(filename);
    viewer->restoreStateFromFile();
    viewer->setStateFileName(QString());
    viewer->update();
    cerr << "Loaded state from " << filename.toStdString() << endl;
  }
}

void MainWindow::on_actionSave_Viewer_State_triggered(bool) {
  QString filename = QFileDialog::getSaveFileName(
      this, "Save State", "camera.xml", "Camera/State file (*.xml)");
  if (!filename.isEmpty()) {
    viewer->setStateFileName(filename);
    viewer->saveStateToFile();
    viewer->setStateFileName(QString());
    cerr << "Saved state to " << filename.toStdString() << endl;
  }
}

void MainWindow::updateRobustKernels() {
  coRobustKernel->clear();
  std::vector<std::string> kernels;
  g2o::RobustKernelFactory::instance()->fillKnownKernels(kernels);
  for (auto& kernel : kernels) {
    coRobustKernel->addItem(QString::fromStdString(kernel));
  }
}
