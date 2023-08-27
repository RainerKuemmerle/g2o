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

#include <memory>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_allocator.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/macros.h"
#include "linear_solver_eigen.h"

namespace g2o {

namespace {
template <int P, int L, bool Blockorder>
std::unique_ptr<BlockSolverBase> AllocateSolver() {
  G2O_DEBUG(
      "Using EigenSparseCholesky poseDim {} landMarkDim {} blockordering {}", P,
      L, Blockorder);
  auto linearSolver = std::make_unique<
      LinearSolverEigen<typename BlockSolverPL<P, L>::PoseMatrixType>>();
  linearSolver->setBlockOrdering(Blockorder);
  return std::make_unique<BlockSolverPL<P, L>>(std::move(linearSolver));
}

/**
 * helper function for allocating
 */
std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  static const OptimizationAlgorithmAllocator::AllocateMap kSolverFactories{
      {"var", &AllocateSolver<-1, -1, true>},
      {"fix3_2", &AllocateSolver<3, 2, true>},
      {"fix6_3", &AllocateSolver<6, 3, true>},
      {"fix7_3", &AllocateSolver<7, 3, true>},
      {"fix3_2_scalar", &AllocateSolver<3, 2, false>},
      {"fix6_3_scalar", &AllocateSolver<6, 3, false>},
      {"fix7_3_scalar", &AllocateSolver<7, 3, false>},
  };

  return OptimizationAlgorithmAllocator::allocate(fullSolverName,
                                                  kSolverFactories);
}
}  // namespace

class EigenSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  explicit EigenSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(eigen);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_var, std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
                "gn_var",
                "Gauss-Newton: Cholesky solver using Eigen's Sparse Cholesky "
                "methods (variable blocksize)",
                "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix3_2,
    std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix3_2",
        "Gauss-Newton: Cholesky solver using  Eigen's Sparse Cholesky (fixed "
        "blocksize)",
        "Eigen", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix6_3,
    std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix6_3",
        "Gauss-Newton: Cholesky solver using  Eigen's Sparse Cholesky (fixed "
        "blocksize)",
        "Eigen", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix7_3,
    std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix7_3",
        "Gauss-Newton: Cholesky solver using  Eigen's Sparse Cholesky (fixed "
        "blocksize)",
        "Eigen", true, 7, 3)));

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_var, std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
                "lm_var",
                "Levenberg: Cholesky solver using Eigen's Sparse Cholesky "
                "methods (variable blocksize)",
                "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix3_2,
    std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix3_2",
        "Levenberg: Cholesky solver using  Eigen's Sparse Cholesky (fixed "
        "blocksize)",
        "Eigen", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix6_3,
    std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix6_3",
        "Levenberg: Cholesky solver using  Eigen's Sparse Cholesky (fixed "
        "blocksize)",
        "Eigen", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix7_3,
    std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix7_3",
        "Levenberg: Cholesky solver using  Eigen's Sparse Cholesky (fixed "
        "blocksize)",
        "Eigen", true, 7, 3)));

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    dl_var, std::make_shared<EigenSolverCreator>(OptimizationAlgorithmProperty(
                "dl_var",
                "Dogleg: Cholesky solver using Eigen's Sparse Cholesky methods "
                "(variable blocksize)",
                "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));
}  // namespace g2o
