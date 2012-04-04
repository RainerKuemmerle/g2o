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

#include <iostream>

#include "main_window.h"
#include "stream_redirect.h"

#include "gui_hyper_graph_action.h"

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/hyper_graph_action.h"

#include "g2o/apps/g2o_cli/g2o_common.h"
#include "g2o/apps/g2o_cli/dl_wrapper.h"

#include "g2o/stuff/command_args.h"

#include <QApplication>
#include <QThread>
using namespace std;
using namespace g2o;

/**
 * \brief helper for calling usleep on any system using Qt
 */
class SleepThread : public QThread
{
  public: // make the proctected methods publicly available
    using QThread::msleep;
    using QThread::usleep;
};

int main(int argc, char** argv)
{
  QApplication qapp(argc, argv);

  string dummy;
  string inputFilename;
  CommandArgs arg;
  arg.param("solverlib", dummy, "", "specify a solver library which will be loaded");
  arg.param("typeslib", dummy, "", "specify a types library which will be loaded");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed", true);

  arg.parseArgs(argc, argv);
  
  // loading the standard solver /  types
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);

  // register all the solvers
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);

  MainWindow mw;
  mw.updateDisplayedSolvers();
  mw.show();

  // redirect the output that normally goes to cerr to the textedit in the viewer
  StreamRedirect redirect(cerr, mw.plainTextEdit);

  // setting up the optimizer
  SparseOptimizer* optimizer = new SparseOptimizer();
  mw.viewer->graph = optimizer;

  // set up the GUI action
  GuiHyperGraphAction guiHyperGraphAction;
  guiHyperGraphAction.viewer = mw.viewer;
  optimizer->addPostIterationAction(&guiHyperGraphAction);

  if (inputFilename.size() > 0) {
    mw.loadFromFile(QString::fromStdString(inputFilename));
  }

  while (mw.isVisible()) {
    guiHyperGraphAction.dumpScreenshots = mw.actionDump_Images->isChecked();
    qapp.processEvents();
    SleepThread::msleep(10);
  }

  delete optimizer;

  // destroy all the singletons
  //Factory::destroy();
  //OptimizationAlgorithmFactory::destroy();
  //HyperGraphActionLibrary::destroy();

  return 0;
}
