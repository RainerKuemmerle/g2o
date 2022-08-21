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

#include "gui_hyper_graph_action.h"

#include <QApplication>
#include <QtGlobal>

#include "g2o_qglviewer.h"

namespace g2o {

bool GuiHyperGraphAction::operator()(
    const HyperGraph& graph,
    const std::shared_ptr<HyperGraphAction::Parameters>& parameters) {
  (void)graph;
  if (viewer) {
    viewer->setUpdateDisplay(true);
    viewer->update();

    if (dumpScreenshots) {
      auto p = std::dynamic_pointer_cast<ParametersIteration>(parameters);
      if (p) {
        viewer->setSnapshotFormat(QString("PNG"));
        viewer->setSnapshotQuality(-1);
#if QT_VERSION < QT_VERSION_CHECK(5, 5, 0)
        viewer->saveSnapshot(QString().sprintf("g2o%.6d.png", p->iteration),
                             true);
#else
        viewer->saveSnapshot(QString::asprintf("g2o%.6d.png", p->iteration),
                             true);
#endif
      }
    }

    qApp->processEvents();
    return true;
  }
  return false;
}

}  // namespace g2o
