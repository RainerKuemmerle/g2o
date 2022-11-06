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

#include "allocate_optimizer.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

namespace g2o {
namespace internal {

using SlamBlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
using SlamLinearSolver = g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

namespace {
std::unique_ptr<g2o::SparseOptimizer> create(bool lm) {
  // Initialize the SparseOptimizer
  auto mOptimizer = std::make_unique<g2o::SparseOptimizer>();
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = g2o::make_unique<SlamBlockSolver>(std::move(linearSolver));
  if (lm) {
    mOptimizer->setAlgorithm(std::shared_ptr<g2o::OptimizationAlgorithm>(
        new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver))));
  } else {
    mOptimizer->setAlgorithm(std::shared_ptr<g2o::OptimizationAlgorithm>(
        new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver))));
  }
  return mOptimizer;
}
}  // namespace

std::unique_ptr<g2o::SparseOptimizer> createOptimizerForTests() {
  return create(false);
}

std::unique_ptr<g2o::SparseOptimizer> createLmOptimizerForTests() {
  return create(true);
}

}  // namespace internal
}  // namespace g2o
