// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_OUTPUT_HELPER_H
#define G2O_OUTPUT_HELPER_H

#include <string>
#include <iosfwd>
#include "g2o/core/hyper_graph.h"

namespace g2o {

  struct OptimizableGraph;

  /**
   * save the state of the optimizer into files for visualizing using Gnuplot
   */
  bool saveGnuplot(const std::string& gnudump, const OptimizableGraph& optimizer);
  bool saveGnuplot(const std::string& gnudump, const HyperGraph::VertexSet& vertices, const HyperGraph::EdgeSet& edges);

  /**
   * dump the edges to a stream, e.g., cout and redirect to gnuplot
   */
  bool dumpEdges(std::ostream& os, const OptimizableGraph& optimizer);

} // end namespace

#endif
