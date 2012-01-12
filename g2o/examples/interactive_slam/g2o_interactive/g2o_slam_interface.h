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

#ifndef G2O_SLAM_INTERFACE_H
#define G2O_SLAM_INTERFACE_H

#include "g2o/core/optimizable_graph.h"
#include "slam_parser/interface/abstract_slam_interface.h"

#include <map>
#include <vector>

namespace g2o {

  class SparseOptimizerOnline;

  class G2oSlamInterface : public SlamParser::AbstractSlamInterface
  {
    public:
      G2oSlamInterface(SparseOptimizerOnline* optimizer);

      bool addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values);

      bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2, const std::vector<double>& measurement, const std::vector<double>& information);

      bool fixNode(const std::vector<int>& nodes);

      bool queryState(const std::vector<int>& nodes);

      bool solveState();

      int updatedGraphEachN() const { return _updateGraphEachN;}
      void setUpdateGraphEachN(int n);

    protected:
      SparseOptimizerOnline* _optimizer;
      bool _firstOptimization;
      int _nodesAdded;
      int _incIterations;
      int _updateGraphEachN;
      int _batchEveryN;
      int _lastBatchStep;
      bool _initSolverDone;

      HyperGraph::VertexSet _verticesAdded;
      HyperGraph::EdgeSet _edgesAdded;

      OptimizableGraph::Vertex* addVertex(int dimension, int id);
      bool printVertex(OptimizableGraph::Vertex* v);
  };

} // end namespace

#endif
