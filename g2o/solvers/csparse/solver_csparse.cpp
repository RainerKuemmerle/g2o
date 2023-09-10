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

#include <Eigen/Core>
#include <memory>
#include <string>
#include <utility>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_allocator.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/stuff/logger.h"
#include "linear_solver_csparse.h"  // IWYU pragma: keep

namespace g2o {
class OptimizationAlgorithm;

namespace {
template <int P, int L, bool Blockorder>
std::unique_ptr<BlockSolverBase> AllocateSolver() {
  G2O_DEBUG("Using CSparse poseDim {} landMarkDim {} blockordering {}", P, L,
            Blockorder);
  auto linearSolver = std::make_unique<
      LinearSolverCSparse<typename BlockSolverPL<P, L>::PoseMatrixType>>();
  linearSolver->setBlockOrdering(Blockorder);
  return std::make_unique<BlockSolverPL<P, L>>(std::move(linearSolver));
}

/**
 * helper function for allocating
 */
std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  static const OptimizationAlgorithmAllocator::AllocateMap kSolverFactories{
      {"var_csparse", &AllocateSolver<-1, -1, true>},
      {"fix3_2_csparse", &AllocateSolver<3, 2, true>},
      {"fix6_3_csparse", &AllocateSolver<6, 3, true>},
      {"fix7_3_csparse", &AllocateSolver<7, 3, true>},
      {"fix3_2_scalar_csparse", &AllocateSolver<3, 2, false>},
      {"fix6_3_scalar_csparse", &AllocateSolver<6, 3, false>},
      {"fix7_3_scalar_csparse", &AllocateSolver<7, 3, false>},
  };

  return OptimizationAlgorithmAllocator::allocate(fullSolverName,
                                                  kSolverFactories);
}

}  // namespace

class CSparseSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  explicit CSparseSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(csparse);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_var_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_var_csparse",
        "Gauss-Newton: Cholesky solver using CSparse (variable blocksize)",
        "CSparse", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix3_2_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix3_2_csparse",
        "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)",
        "CSparse", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix6_3_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix6_3_csparse",
        "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)",
        "CSparse", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix7_3_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix7_3_csparse",
        "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)",
        "CSparse", true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_var_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_var_csparse",
        "Levenberg: Cholesky solver using CSparse (variable blocksize)",
        "CSparse", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix3_2_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix3_2_csparse",
        "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse",
        true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix6_3_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix6_3_csparse",
        "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse",
        true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix7_3_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix7_3_csparse",
        "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse",
        true, 7, 3)));

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    dl_var_csparse,
    std::make_shared<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "dl_var_csparse",
        "Dogleg: Cholesky solver using CSparse (variable blocksize)", "CSparse",
        false, Eigen::Dynamic, Eigen::Dynamic)));

}  // namespace g2o
