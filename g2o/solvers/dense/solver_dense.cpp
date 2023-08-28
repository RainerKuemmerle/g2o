// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include <memory>

#include "g2o/config.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_allocator.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/solver.h"
#include "g2o/stuff/logger.h"
#include "linear_solver_dense.h"

namespace g2o {
namespace {
template <int P, int L>
std::unique_ptr<g2o::Solver> AllocateSolver() {
  G2O_DEBUG("Using DENSE poseDim {} landMarkDim {}", P, L);
  return std::make_unique<BlockSolverPL<P, L>>(
      std::make_unique<
          LinearSolverDense<typename BlockSolverPL<P, L>::PoseMatrixType>>());
}

std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  static const OptimizationAlgorithmAllocator::AllocateMap kSolverFactories{
      {"dense", &AllocateSolver<-1, -1>},
      {"dense3_2", &AllocateSolver<3, 2>},
      {"dense6_3", &AllocateSolver<6, 3>},
      {"dense7_3", &AllocateSolver<7, 3>},
  };

  return OptimizationAlgorithmAllocator::allocate(fullSolverName,
                                                  kSolverFactories);
}

}  // namespace

class DenseSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  explicit DenseSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(dense);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense,
    std::make_shared<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_dense", "Gauss-Newton: Dense solver (variable blocksize)", "Dense",
        false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense3_2,
    std::make_shared<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_dense3_2", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense6_3,
    std::make_shared<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_dense6_3", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense7_3,
    std::make_shared<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_dense7_3", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense,
    std::make_shared<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_dense", "Levenberg: Dense solver (variable blocksize)", "Dense",
        false, -1, -1)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense3_2,
    std::make_shared<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_dense3_2", "Levenberg: Dense solver (fixed blocksize)", "Dense",
        true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense6_3,
    std::make_shared<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_dense6_3", "Levenberg: Dense solver (fixed blocksize)", "Dense",
        true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense7_3,
    std::make_shared<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_dense7_3", "Levenberg: Dense solver (fixed blocksize)", "Dense",
        true, 7, 3)));

}  // namespace g2o
