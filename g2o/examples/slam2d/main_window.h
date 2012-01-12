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

#include "ui_base_main_window.h"

namespace g2o {
  class OptimizationAlgorithm;
}

class MainWindow : public QMainWindow, public Ui::BaseMainWindow
{
  Q_OBJECT
  public:
    MainWindow(QWidget * parent = 0, Qt::WindowFlags flags = 0);
    ~MainWindow();

    g2o::OptimizationAlgorithm* solverGaussNewton;
    g2o::OptimizationAlgorithm* solverLevenberg;

  public slots:
    void on_actionLoad_triggered(bool);
    void on_actionSave_triggered(bool);
    void on_actionQuit_triggered(bool);
    void on_btnOptimize_clicked();
    void on_btnInitialGuess_clicked();

  protected:
    void fixGraph();

};

#endif
