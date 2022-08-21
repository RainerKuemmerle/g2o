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

#ifndef G2O_GUI_HYPER_GRAPH_ACTION_H
#define G2O_GUI_HYPER_GRAPH_ACTION_H

#include "g2o/core/hyper_graph_action.h"
#include "g2o_viewer_api.h"

namespace g2o {

class G2oQGLViewer;

/**
 * \brief action which calls an GUI update after each iteration
 */
class G2O_VIEWER_API GuiHyperGraphAction : public HyperGraphAction {
 public:
  /**
   * calling updateGL, processEvents to visualize the current state after each
   * iteration
   */
  bool operator()(const HyperGraph& graph,
                  const std::shared_ptr<HyperGraphAction::Parameters>&
                      parameters = nullptr) override;

  G2oQGLViewer* viewer = nullptr;  ///< the viewer which visualizes the graph
  bool dumpScreenshots = false;
};

}  // namespace g2o

#endif
