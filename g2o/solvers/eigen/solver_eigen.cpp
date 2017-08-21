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

using namespace std;

namespace g2o {

  namespace
  {
    template<int p, int l, bool blockorder>
    std::unique_ptr<BlockSolverBase> AllocateSolver()
    {
      std::cerr << "# Using EigenSparseCholesky poseDim " << p << " landMarkDim " << l << " blockordering " << blockorder << std::endl;
      auto linearSolver = g2o::make_unique<LinearSolverEigen<typename BlockSolverPL<p, l>::PoseMatrixType>>();
      linearSolver->setBlockOrdering(blockorder);
      return g2o::make_unique<BlockSolverPL<p, l>>(std::move(linearSolver));
    }
  }

  /**
   * helper function for allocating
   */
  static OptimizationAlgorithm* createSolver(const std::string& fullSolverName)
  {
    static const std::map<std::string, std::function<std::unique_ptr<BlockSolverBase>()>> solver_factories{
      { "var_eigen", &AllocateSolver<-1, -1, true> },
      { "fix3_2_eigen", &AllocateSolver<3, 2, true> },
      { "fix6_3_eigen", &AllocateSolver<6, 3, true> },
      { "fix7_3_eigen", &AllocateSolver<7, 3, true> },
      { "fix3_2_scalar_eigen", &AllocateSolver<3, 2, false> },
      { "fix6_3_scalar_eigen", &AllocateSolver<6, 3, false> },
      { "fix7_3_scalar_eigen", &AllocateSolver<7, 3, false> },
    };

    string solverName = fullSolverName.substr(3);
    auto solverf = solver_factories.find(solverName);
    if (solverf == solver_factories.end())
      return nullptr;

    string methodName = fullSolverName.substr(0, 2);

    if (methodName == "gn")
    {
      return new OptimizationAlgorithmGaussNewton(solverf->second());
    }
    else if (methodName == "lm")
    {
      return new OptimizationAlgorithmLevenberg(solverf->second());
    }
    else if (methodName == "dl")
    {
      return new OptimizationAlgorithmDogleg(solverf->second());
    }

    return nullptr;
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
