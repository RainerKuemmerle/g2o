// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_SLAM_INTERFACE_H
#define G2O_SLAM_INTERFACE_H

#include "g2o_interactive_api.h"
#include "g2o/core/optimizable_graph.h"
#include "slam_parser/interface/abstract_slam_interface.h"

#include <map>
#include <vector>

namespace g2o {

  class SparseOptimizerOnline;

  class G2O_INTERACTIVE_API G2oSlamInterface : public SlamParser::AbstractSlamInterface
  {
    public:
      enum SolveResult { SOLVED, SOLVED_BATCH, NOOP, ERROR };

    public:
      G2oSlamInterface(SparseOptimizerOnline* optimizer);

      bool addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values);

      bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2, const std::vector<double>& measurement, const std::vector<double>& information);

      bool fixNode(const std::vector<int>& nodes);

      bool queryState(const std::vector<int>& nodes);

      bool solveState();

      SolveResult solve();

      int updatedGraphEachN() const { return _updateGraphEachN;}
      void setUpdateGraphEachN(int n);

      int batchSolveEachN() const { return _batchEveryN;}
      void setBatchSolveEachN(int n);

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
