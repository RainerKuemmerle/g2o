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
#include "viewer_properties_widget.h"

#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/estimate_propagator.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/robust_kernel.h"

#include <QFileDialog>
#include <QStandardItemModel>
#include <QDoubleValidator>
#include <QComboBox>

#include <fstream>
#include <iostream>
#include <cassert>
using namespace std;
using namespace g2o;

MainWindow::MainWindow(QWidget * parent, Qt::WindowFlags flags) :
  QMainWindow(parent, flags),
  _lastSolver(-1), _currentSolver(0), _viewerPropertiesWidget(0), _optimizerPropertiesWidget(0),
  _filename("")
{
  setupUi(this);
  leKernelWidth->setValidator(new QDoubleValidator(-numeric_limits<double>::max(), numeric_limits<double>::max(), 7, this));
  plainTextEdit->setMaximumBlockCount(1000);
  btnForceStop->hide();
  QObject::connect(cbDrawAxis, SIGNAL(toggled(bool)), viewer, SLOT(setAxisIsDrawn(bool)));
}

MainWindow::~MainWindow()
{
}

void MainWindow::on_actionLoad_triggered(bool)
{
  QString filename = QFileDialog::getOpenFileName(this, "Load g2o file", "", "g2o files (*.g2o);;All Files (*)");
  if (! filename.isNull()) {
    loadFromFile(filename);
  }
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

void MainWindow::on_btnOptimize_clicked()
{
  if (viewer->graph->vertices().size() == 0 || viewer->graph->edges().size() == 0) {
    cerr << "Graph has no vertices / egdes" << endl;
    return;
  }

  bool allocatedNewSolver;
  bool allocateStatus = allocateSolver(allocatedNewSolver);
  if (! allocateStatus) {
    cerr << "Error while allocating solver" << endl;
    return;
  }
  if (allocatedNewSolver)
    prepare();
  setRobustKernel();

  btnOptimize->hide();
  btnForceStop->show();

  _forceStopFlag = false;
  viewer->graph->setForceStopFlag(&_forceStopFlag);

  int maxIterations = spIterations->value();
  int iter = viewer->graph->optimize(maxIterations);
  if (maxIterations > 0 && !iter){
    cerr << "Optimization failed, result might be invalid" << endl;
  }

  btnOptimize->show();
  btnForceStop->hide();

  viewer->setUpdateDisplay(true);
  viewer->update();
  _forceStopFlag = false;
}

void MainWindow::on_btnInitialGuess_clicked()
{
  if (viewer->graph->activeEdges().size() == 0)
    viewer->graph->initializeOptimization();

  switch (cbxIniitialGuessMethod->currentIndex()) {
    case 0:
      // spanning tree
      viewer->graph->computeInitialGuess();
      break;
    case 1:
      // odometry
      {
        EstimatePropagatorCostOdometry costFunction(viewer->graph);
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

void MainWindow::on_btnSetZero_clicked()
{
  if (viewer->graph->activeEdges().size() == 0)
    viewer->graph->initializeOptimization();

  viewer->graph->setToOrigin();
  viewer->setUpdateDisplay(true);
  viewer->update();
}

void MainWindow::on_btnReload_clicked()
{
  if (_filename.length()>0){
    cerr << "reloading " << _filename << endl;
    viewer->graph->clear();
    viewer->graph->load(_filename.c_str());
    viewer->setUpdateDisplay(true);
    viewer->update();
  }
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
    cerr << "graph is fixed by priors or nodes are already fixed" << endl;
  }

  viewer->graph->setVerbose(true);
  //viewer->graph->computeActiveErrors();
}

void MainWindow::on_actionQuit_triggered(bool)
{
  close();
}

void MainWindow::updateDisplayedSolvers()
{
  coOptimizer->clear();
  _knownSolvers.clear();
  const OptimizationAlgorithmFactory::CreatorList& knownSolvers = OptimizationAlgorithmFactory::instance()->creatorList();

  bool varFound = false;
  string varType = "";
  for (OptimizationAlgorithmFactory::CreatorList::const_iterator it = knownSolvers.begin(); it != knownSolvers.end(); ++it) {
    const OptimizationAlgorithmProperty& sp = (*it)->property();
    if (sp.name == "gn_var" || sp.name == "gn_var_cholmod") {
      varType = sp.type;
      varFound = true;
      break;
    }
  }

  if (varFound) {
    for (OptimizationAlgorithmFactory::CreatorList::const_iterator it = knownSolvers.begin(); it != knownSolvers.end(); ++it) {
      const OptimizationAlgorithmProperty& sp = (*it)->property();
      if (sp.type == varType) {
        coOptimizer->addItem(QString::fromStdString(sp.name));
        _knownSolvers.push_back(sp);
      }
    }
  }

  map<string, vector<OptimizationAlgorithmProperty> > solverLookUp;

  for (OptimizationAlgorithmFactory::CreatorList::const_iterator it = knownSolvers.begin(); it != knownSolvers.end(); ++it) {
    const OptimizationAlgorithmProperty& sp = (*it)->property();
    if (varFound && varType == sp.type)
      continue;
    solverLookUp[sp.type].push_back(sp);
  }

  for (map<string, vector<OptimizationAlgorithmProperty> >::iterator it = solverLookUp.begin(); it != solverLookUp.end(); ++it) {
    if (_knownSolvers.size() > 0) {
      coOptimizer->insertSeparator(coOptimizer->count());
      _knownSolvers.push_back(OptimizationAlgorithmProperty());
    }
    const vector<OptimizationAlgorithmProperty>& vsp = it->second;
    for (size_t j = 0; j < vsp.size(); ++j) {
      coOptimizer->addItem(QString::fromStdString(vsp[j].name));
      _knownSolvers.push_back(vsp[j]);
    }
  }
}

bool MainWindow::load(const QString& filename)
{
  viewer->graph->clear();
  bool loadStatus = false;
  if (filename == "-") {
    cerr << "reading stdin" << endl;
    loadStatus = viewer->graph->load(cin);
  } else {
    ifstream ifs(filename.toStdString().c_str());
    if (! ifs)
      return false;
    loadStatus = viewer->graph->load(ifs);
  }
  if (! loadStatus)
    return false;
  _lastSolver = -1;
  viewer->setUpdateDisplay(true);
  SparseOptimizer* optimizer = viewer->graph;

  // update the solvers which are suitable for this graph
  set<int> vertDims = optimizer->dimensions();
  for (size_t i = 0; i < _knownSolvers.size(); ++i) {
    const OptimizationAlgorithmProperty& sp = _knownSolvers[i];
    if (sp.name == "" && sp.desc == "")
      continue;

    bool suitableSolver = optimizer->isSolverSuitable(sp, vertDims);
    qobject_cast<QStandardItemModel *>(coOptimizer->model())->item(i)->setEnabled(suitableSolver);
  }
  return loadStatus;
}

bool MainWindow::allocateSolver(bool& allocatedNewSolver)
{
  if (coOptimizer->count() == 0) {
    cerr << "No solvers available" << endl;
    return false;
  }
  int currentIndex = coOptimizer->currentIndex();
  bool enabled = qobject_cast<QStandardItemModel *>(coOptimizer->model())->item(currentIndex)->isEnabled();

  if (! enabled) {
    cerr << "selected solver is not enabled" << endl;
    return false;
  }

  if (currentIndex == _lastSolver)
    return true;

  allocatedNewSolver = true;
  QString strSolver = coOptimizer->currentText();

  // delete the old optimization algorithm
  OptimizationAlgorithm* algorithmPointer = const_cast<OptimizationAlgorithm*>(viewer->graph->algorithm());
  viewer->graph->setAlgorithm(0);
  delete algorithmPointer;

  // create the new algorithm
  OptimizationAlgorithmFactory* solverFactory = OptimizationAlgorithmFactory::instance();
  _currentSolver = solverFactory->construct(strSolver.toStdString(), _currentOptimizationAlgorithmProperty);
  viewer->graph->setAlgorithm(_currentSolver);

  _lastSolver = currentIndex;
  return true;
}

bool MainWindow::prepare()
{
  SparseOptimizer* optimizer = viewer->graph;
  if (_currentOptimizationAlgorithmProperty.requiresMarginalize) {
    cerr << "Marginalizing Landmarks" << endl;
    for (SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
      int vdim = v->dimension();
      v->setMarginalized((vdim == _currentOptimizationAlgorithmProperty.landmarkDim));
    }
  }
  else {
    cerr << "Preparing (no marginalization of Landmarks)" << endl;
    for (SparseOptimizer::VertexIDMap::const_iterator it = optimizer->vertices().begin(); it != optimizer->vertices().end(); ++it) {
      OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(it->second);
      v->setMarginalized(false);
    }
  }
  viewer->graph->initializeOptimization();
  return true;
}

void MainWindow::setRobustKernel()
{
  SparseOptimizer* optimizer = viewer->graph;
  bool robustKernel = cbRobustKernel->isChecked();
  double huberWidth = leKernelWidth->text().toDouble();
  //odometry edges are those whose node ids differ by 1

  bool onlyLoop = cbOnlyLoop->isChecked();

  if (robustKernel) {
    QString strRobustKernel = coRobustKernel->currentText();
    AbstractRobustKernelCreator* creator = RobustKernelFactory::instance()->creator(strRobustKernel.toStdString());
    if (! creator) {
      cerr << strRobustKernel.toStdString() << " is not a valid robust kernel" << endl;
      return;
    }
    for (SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      if (onlyLoop) {
        if (e->vertices().size() >= 2 && std::abs(e->vertex(0)->id() - e->vertex(1)->id()) != 1) {
          e->setRobustKernel(creator->construct());
          e->robustKernel()->setDelta(huberWidth);
        }
      } else {
        e->setRobustKernel(creator->construct());
        e->robustKernel()->setDelta(huberWidth);
      }
    }
  } else {
    for (SparseOptimizer::EdgeSet::const_iterator it = optimizer->edges().begin(); it != optimizer->edges().end(); ++it) {
      OptimizableGraph::Edge* e = static_cast<OptimizableGraph::Edge*>(*it);
      e->setRobustKernel(0);
    }
  }
}

void MainWindow::on_btnForceStop_clicked()
{
  _forceStopFlag = true;
}

bool MainWindow::loadFromFile(const QString& filename)
{
  viewer->graph->clear();
  bool loadStatus = load(filename);
  if (loadStatus){
    _filename = filename.toStdString();
  }
  cerr << "loaded " << filename.toStdString() << " with " << viewer->graph->vertices().size()
    << " vertices and " << viewer->graph->edges().size() << " measurements" << endl;
  viewer->update();
  fixGraph();
  return loadStatus;
}

void MainWindow::on_actionWhite_Background_triggered(bool)
{
  viewer->setBackgroundColor(QColor::fromRgb(255, 255, 255));
  viewer->update();
}

void MainWindow::on_actionDefault_Background_triggered(bool)
{
  viewer->setBackgroundColor(QColor::fromRgb(51, 51, 51));
  viewer->update();
}

void MainWindow::on_actionProperties_triggered(bool)
{
  if (! _viewerPropertiesWidget) {
    _viewerPropertiesWidget = new ViewerPropertiesWidget(this);
    _viewerPropertiesWidget->setWindowTitle(tr("Drawing Options"));
  }
  _viewerPropertiesWidget->setViewer(viewer);
  _viewerPropertiesWidget->show();
}

void MainWindow::on_btnOptimizerParamaters_clicked()
{
  if (! _optimizerPropertiesWidget) {
    _optimizerPropertiesWidget = new PropertiesWidget(this);
    _optimizerPropertiesWidget->setWindowTitle(tr("Internal Solver Properties"));
  }
  bool allocatedNewSolver;
  bool allocateStatus = allocateSolver(allocatedNewSolver);
  if (! allocateStatus) {
    cerr << "Error while allocating solver" << endl;
    return;
  }
  if (allocatedNewSolver)
    prepare();
  if (_currentSolver) {
    _optimizerPropertiesWidget->setProperties(const_cast<g2o::PropertyMap*>(&_currentSolver->properties()));
  } else {
    _optimizerPropertiesWidget->setProperties(0);
  }
  _optimizerPropertiesWidget->show();
}

void MainWindow::on_actionSave_Screenshot_triggered(bool)
{
  QString selectedFilter;
  QString filename = QFileDialog::getSaveFileName(this, "Save screen to a file", "viewer.png",
      "PNG files (*.png);;JPG files (*.jpg);;EPS files (*.eps)", &selectedFilter);

  if (! filename.isNull()) {
    // extract the file format from the filter options
    int spacePos = selectedFilter.indexOf(' ');
    assert(spacePos > 0 && "extracting the image format failed");
    QString format = selectedFilter.left(spacePos);
    // setting up the snapshot and save to file
    if (format == "JPG") {
      viewer->setSnapshotQuality(90);
    } else {
      viewer->setSnapshotQuality(-1);
    }
    viewer->setSnapshotFormat(format);
    viewer->saveSnapshot(filename);
    cerr << "saved snapshot " << filename.toStdString() << "(" << format.toStdString() << ")" << endl;
  }
}

void MainWindow::on_actionLoad_Viewer_State_triggered(bool)
{
  QString filename = QFileDialog::getOpenFileName(this, "Load State", "camera.xml", "Camera/State file (*.xml)");
  if (!filename.isEmpty()) {
    viewer->setStateFileName(filename);
    viewer->restoreStateFromFile();
    viewer->setStateFileName(QString());
    viewer->update();
    cerr << "Loaded state from " << filename.toStdString() << endl;
  }
}

void MainWindow::on_actionSave_Viewer_State_triggered(bool)
{
  QString filename = QFileDialog::getSaveFileName(this, "Save State", "camera.xml", "Camera/State file (*.xml)");
  if (!filename.isEmpty()) {
    viewer->setStateFileName(filename);
    viewer->saveStateToFile();
    viewer->setStateFileName(QString());
    cerr << "Saved state to " << filename.toStdString() << endl;
  }
}

void MainWindow::updateRobustKernels()
{
  coRobustKernel->clear();
  std::vector<std::string> kernels;
  RobustKernelFactory::instance()->fillKnownKernels(kernels);
  for (size_t i = 0; i < kernels.size(); ++i) {
    coRobustKernel->addItem(QString::fromStdString(kernels[i]));
  }
}
