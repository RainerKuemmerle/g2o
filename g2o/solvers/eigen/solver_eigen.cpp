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

#include "linear_solver_eigen.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/stuff/macros.h"

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_EIGEN_SPARSE_CHOLESKY(s, p, l, blockorder) \
  if (1) { \
    std::cerr << "# Using EigenSparseCholesky poseDim " << p << " landMarkDim " << l << " blockordering " << blockorder << std::endl; \
    LinearSolverEigen< DIM_TO_SOLVER(p, l)::PoseMatrixType >* linearSolver = new LinearSolverEigen<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
    linearSolver->setBlockOrdering(blockorder); \
    s = new DIM_TO_SOLVER(p, l)(linearSolver); \
  } else (void)0

namespace g2o {

  /**
   * helper function for allocating
   */
  static OptimizationAlgorithm* createSolver(const std::string& fullSolverName)
  {
    g2o::Solver* s = 0;

    string methodName = fullSolverName.substr(0, 2);
    string solverName = fullSolverName.substr(3);

    if (solverName == "var_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, -1, -1, true);
    }
#if 0
    else if (solverName == "fix3_2_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 3, 2, true);
    }
    else if (solverName == "fix6_3_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 6, 3, true);
    }
    else if (solverName == "fix7_3_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 7, 3, true);
    }
    else if (solverName == "fix3_2_scalar_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 3, 2, false);
    }
    else if (solverName == "fix6_3_scalar_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 6, 3, false);
    }
    else if (solverName == "fix7_3_scalar_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 7, 3, false);
    }
#endif

    OptimizationAlgorithm* snl = 0;
    if (methodName == "gn") {
      snl = new OptimizationAlgorithmGaussNewton(s);
    }
    else if (methodName == "lm") {
      snl = new OptimizationAlgorithmLevenberg(s);
    }
    else if (methodName == "dl") {
      BlockSolverBase* blockSolver = dynamic_cast<BlockSolverBase*>(s);
      snl = new OptimizationAlgorithmDogleg(blockSolver);
    }

    return snl;
  }

  class EigenSolverCreator : public AbstractOptimizationAlgorithmCreator
  {
    public:
      EigenSolverCreator(const OptimizationAlgorithmProperty& p) : AbstractOptimizationAlgorithmCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };


  G2O_REGISTER_OPTIMIZATION_LIBRARY(eigen);

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_var_eigen, new EigenSolverCreator(OptimizationAlgorithmProperty("gn_var_eigen", "Gauss-Newton: Cholesky solver using Eigen's Sparse Cholesky methods (variable blocksize)", "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_var_eigen, new EigenSolverCreator(OptimizationAlgorithmProperty("lm_var_eigen", "Levenberg: Cholesky solver using Eigen's Sparse Cholesky methods (variable blocksize)", "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(dl_var_eigen, new EigenSolverCreator(OptimizationAlgorithmProperty("dl_var_eigen", "Dogleg: Cholesky solver using Eigen's Sparse Cholesky methods (variable blocksize)", "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));
}
