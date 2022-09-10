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

#include <map>
#include <vector>

#include "g2o/core/optimizable_graph.h"
#include "g2o_interactive_api.h"
#include "slam_parser/interface/abstract_slam_interface.h"

namespace g2o {

class SparseOptimizerOnline;

class G2O_INTERACTIVE_API G2oSlamInterface
    : public slam_parser::AbstractSlamInterface {
 public:
  enum SolveResult { kSolved, kSolvedBatch, kNoop, kError };

  explicit G2oSlamInterface(SparseOptimizerOnline* optimizer);

  bool addNode(const std::string& tag, int id, int dimension,
               const std::vector<double>& values) override;

  bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2,
               const std::vector<double>& measurement,
               const std::vector<double>& information) override;

  bool fixNode(const std::vector<int>& nodes) override;

  bool queryState(const std::vector<int>& nodes) override;

  bool solveState() override;

  SolveResult solve();

  int updatedGraphEachN() const { return updateGraphEachN_; }
  void setUpdateGraphEachN(int n);

  int batchSolveEachN() const { return batchEveryN_; }
  void setBatchSolveEachN(int n);

  SparseOptimizerOnline* optimizer() { return optimizer_; }

 protected:
  SparseOptimizerOnline* optimizer_;
  bool firstOptimization_ = true;
  int nodesAdded_ = 0;
  int incIterations_ = 1;
  int updateGraphEachN_ = 10;
  int batchEveryN_ = 100;
  int lastBatchStep_ = 0;
  bool initSolverDone_ = false;

  HyperGraph::VertexSet verticesAdded_;
  HyperGraph::EdgeSet edgesAdded_;

  std::shared_ptr<OptimizableGraph::Vertex> addVertex(int dimension, int id);
  static bool printVertex(OptimizableGraph::Vertex* v);
};

}  // namespace g2o

#endif
