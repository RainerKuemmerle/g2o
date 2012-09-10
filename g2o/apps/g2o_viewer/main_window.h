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

#ifndef G2O_MAIN_WINDOW_H
#define G2O_MAIN_WINDOW_H

#include "g2o_viewer_api.h"
#include "ui_base_main_window.h"

#include "g2o/core/optimization_algorithm_property.h"

#include <vector>

class ViewerPropertiesWidget;
class PropertiesWidget;

namespace g2o {
  class DlWrapper;
  class OptimizationAlgorithm;
} // end namespace

/**
 * \brief main window of the g2o viewer
 */
class G2O_VIEWER_API MainWindow : public QMainWindow, public Ui::BaseMainWindow
{
  Q_OBJECT
  public:
    MainWindow(QWidget * parent = 0, Qt::WindowFlags flags = 0);
    ~MainWindow();

  public:
    /**
     * list the available solvers in the GUI
     */
    void updateDisplayedSolvers();

    /**
     * list the available robust kernels in the GUI
     */
    void updateRobustKernels();

    /**
     * load a graph on which we will operate from a file
     */
    bool loadFromFile(const QString& filename);

  public slots:
    void on_actionLoad_triggered(bool);
    void on_actionSave_triggered(bool);
    void on_actionQuit_triggered(bool);
    void on_actionWhite_Background_triggered(bool);
    void on_actionDefault_Background_triggered(bool);
    void on_actionProperties_triggered(bool);
    void on_actionSave_Screenshot_triggered(bool);
    void on_actionLoad_Viewer_State_triggered(bool);
    void on_actionSave_Viewer_State_triggered(bool);

    void on_btnOptimize_clicked();
    void on_btnInitialGuess_clicked();
    void on_btnSetZero_clicked();
    void on_btnForceStop_clicked();
    void on_btnOptimizerParamaters_clicked();

  protected:
    void fixGraph();
    bool allocateSolver(bool& allocatedNewSolver);
    bool prepare();
    void setRobustKernel();
    bool load(const QString& filename);

    std::vector<g2o::OptimizationAlgorithmProperty> _knownSolvers;
    int _lastSolver;
    bool _forceStopFlag;
    g2o::OptimizationAlgorithmProperty _currentOptimizationAlgorithmProperty;
    g2o::OptimizationAlgorithm* _currentSolver;

    ViewerPropertiesWidget* _viewerPropertiesWidget;
    PropertiesWidget* _optimizerPropertiesWidget;
};


#endif
