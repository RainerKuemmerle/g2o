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

#include "optimization_algorithm_with_hessian.h"

#include <iostream>

#include "optimizable_graph.h"
#include "solver.h"
#include "sparse_optimizer.h"

namespace g2o {

OptimizationAlgorithmWithHessian::OptimizationAlgorithmWithHessian(
    Solver& solver)
    : solver_(solver) {
  writeDebug_ = properties_.makeProperty<Property<bool> >("writeDebug", true);
}

bool OptimizationAlgorithmWithHessian::init(bool online) {
  assert(optimizer_ && "optimizer_ not set");
  solver_.setWriteDebug(writeDebug_->value());
  bool useSchur = false;
  for (const auto& v : optimizer_->activeVertices()) {
    if (v->marginalized()) {
      useSchur = true;
      break;
    }
  }
  if (useSchur) {
    if (solver_.supportsSchur()) solver_.setSchur(true);
  } else {
    if (solver_.supportsSchur()) solver_.setSchur(false);
  }

  const bool initState = solver_.init(optimizer_, online);
  return initState;
}

bool OptimizationAlgorithmWithHessian::computeMarginals(
    SparseBlockMatrix<MatrixX>& spinv,
    const std::vector<std::pair<int, int> >& blockIndices) {
  return solver_.computeMarginals(spinv, blockIndices);
}

bool OptimizationAlgorithmWithHessian::buildLinearStructure() {
  return solver_.buildStructure();
}

void OptimizationAlgorithmWithHessian::updateLinearSystem() {
  solver_.buildSystem();
}

bool OptimizationAlgorithmWithHessian::updateStructure(
    const HyperGraph::VertexContainer& vset, const HyperGraph::EdgeSet& edges) {
  return solver_.updateStructure(vset, edges);
}

void OptimizationAlgorithmWithHessian::setWriteDebug(bool writeDebug) {
  writeDebug_->setValue(writeDebug);
}

}  // namespace g2o
