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

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/macros.h"
#include "linear_solver_eigen.h"

using namespace std;

namespace g2o {

namespace {
template <int p, int l, bool blockorder>
std::unique_ptr<BlockSolverBase> AllocateSolver() {
  G2O_DEBUG(
      "Using EigenSparseCholesky poseDim {} landMarkDim {} blockordering {}", p,
      l, blockorder);
  auto linearSolver = std::make_unique<
      LinearSolverEigen<typename BlockSolverPL<p, l>::PoseMatrixType>>();
  linearSolver->setBlockOrdering(blockorder);
  return std::make_unique<BlockSolverPL<p, l>>(std::move(linearSolver));
}
}  // namespace

/**
 * helper function for allocating
 */
static OptimizationAlgorithm* createSolver(const std::string& fullSolverName) {
  static const std::map<std::string,
                        std::function<std::unique_ptr<BlockSolverBase>()>>
      solver_factories{
          {"var", &AllocateSolver<-1, -1, true>},
          {"fix3_2", &AllocateSolver<3, 2, true>},
          {"fix6_3", &AllocateSolver<6, 3, true>},
          {"fix7_3", &AllocateSolver<7, 3, true>},
          {"fix3_2_scalar", &AllocateSolver<3, 2, false>},
          {"fix6_3_scalar", &AllocateSolver<6, 3, false>},
          {"fix7_3_scalar", &AllocateSolver<7, 3, false>},
      };

  string solverName = fullSolverName.substr(3);
  auto solverf = solver_factories.find(solverName);
  if (solverf == solver_factories.end()) return nullptr;

  string methodName = fullSolverName.substr(0, 2);

  if (methodName == "gn") {
    return new OptimizationAlgorithmGaussNewton(solverf->second());
  } else if (methodName == "lm") {
    return new OptimizationAlgorithmLevenberg(solverf->second());
  } else if (methodName == "dl") {
    return new OptimizationAlgorithmDogleg(solverf->second());
  }

  return nullptr;
}

class EigenSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  explicit EigenSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  virtual OptimizationAlgorithm* construct() {
    return createSolver(property().name);
  }
};

// clang-format off
  G2O_REGISTER_OPTIMIZATION_LIBRARY(eigen);

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_var, new EigenSolverCreator(OptimizationAlgorithmProperty("gn_var", "Gauss-Newton: Cholesky solver using Eigen's Sparse Cholesky methods (variable blocksize)", "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix3_2, new EigenSolverCreator(OptimizationAlgorithmProperty("gn_fix3_2", "Gauss-Newton: Cholesky solver using  Eigen's Sparse Cholesky (fixed blocksize)", "Eigen", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix6_3, new EigenSolverCreator(OptimizationAlgorithmProperty("gn_fix6_3", "Gauss-Newton: Cholesky solver using  Eigen's Sparse Cholesky (fixed blocksize)", "Eigen", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix7_3, new EigenSolverCreator(OptimizationAlgorithmProperty("gn_fix7_3", "Gauss-Newton: Cholesky solver using  Eigen's Sparse Cholesky (fixed blocksize)", "Eigen", true, 7, 3)));

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_var, new EigenSolverCreator(OptimizationAlgorithmProperty("lm_var", "Levenberg: Cholesky solver using Eigen's Sparse Cholesky methods (variable blocksize)", "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix3_2, new EigenSolverCreator(OptimizationAlgorithmProperty("lm_fix3_2", "Levenberg: Cholesky solver using  Eigen's Sparse Cholesky (fixed blocksize)", "Eigen", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix6_3, new EigenSolverCreator(OptimizationAlgorithmProperty("lm_fix6_3", "Levenberg: Cholesky solver using  Eigen's Sparse Cholesky (fixed blocksize)", "Eigen", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix7_3, new EigenSolverCreator(OptimizationAlgorithmProperty("lm_fix7_3", "Levenberg: Cholesky solver using  Eigen's Sparse Cholesky (fixed blocksize)", "Eigen", true, 7, 3)));

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(dl_var, new EigenSolverCreator(OptimizationAlgorithmProperty("dl_var", "Dogleg: Cholesky solver using Eigen's Sparse Cholesky methods (variable blocksize)", "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));
// clang-format on
}  // namespace g2o
