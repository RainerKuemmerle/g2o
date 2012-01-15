// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "solver_dense.h"
#include "linear_solver_dense.h"

#include "g2o/config.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_factory.h"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_DENSE(s, p, l) \
  if (1) { \
      std::cerr << "# Using DENSE poseDim " << p << " landMarkDim " << l << std::endl; \
      DIM_TO_SOLVER(p, l)::LinearSolverType* linearSolver = new LinearSolverDense<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
      s = new DIM_TO_SOLVER(p, l)(linearSolver); \
  } else (void)0

namespace g2o {

  static OptimizationAlgorithm* createSolver(const std::string& fullSolverName)
  {
    g2o::Solver* s = 0;

    string methodName = fullSolverName.substr(0, 2);
    string solverName = fullSolverName.substr(3);

    if (solverName == "dense") {
      ALLOC_DENSE(s, -1, -1);
    }
    else if (solverName == "dense3_2") {
      ALLOC_DENSE(s, 3, 2);
    }
    else if (solverName == "dense6_3") {
      ALLOC_DENSE(s, 6, 3);
    }
    else if (solverName == "dense7_3") {
      ALLOC_DENSE(s, 7, 3);
    }

    OptimizationAlgorithm* snl = 0;
    if (methodName == "gn") {
      snl = new OptimizationAlgorithmGaussNewton(s);
    }
    else if (methodName == "lm") {
      snl = new OptimizationAlgorithmLevenberg(s);
    }

    return snl;
  }

  class DenseSolverCreator : public AbstractOptimizationAlgorithmCreator
  {
    public:
      DenseSolverCreator(const OptimizationAlgorithmProperty& p) : AbstractOptimizationAlgorithmCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };

  G2O_ATTRIBUTE_CONSTRUCTOR(init_solver_csparse)
  {
    solver_dense::init();
  }

  namespace solver_dense {
    void init()
    {
      static bool initialized;
      if (initialized)
        return;
      OptimizationAlgorithmFactory* factory = OptimizationAlgorithmFactory::instance();
      factory->registerSolver(new DenseSolverCreator(OptimizationAlgorithmProperty("gn_dense", "Gauss-Newton: Dense solver (variable blocksize)", "Dense", false, -1, -1)));
      factory->registerSolver(new DenseSolverCreator(OptimizationAlgorithmProperty("gn_dense3_2", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense", true, 3, 2)));
      factory->registerSolver(new DenseSolverCreator(OptimizationAlgorithmProperty("gn_dense6_3", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense", true, 6, 3)));
      factory->registerSolver(new DenseSolverCreator(OptimizationAlgorithmProperty("gn_dense7_3", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense", true, 7, 3)));
      factory->registerSolver(new DenseSolverCreator(OptimizationAlgorithmProperty("lm_dense", "Levenberg: Dense solver (variable blocksize)", "Dense", false, -1, -1)));
      factory->registerSolver(new DenseSolverCreator(OptimizationAlgorithmProperty("lm_dense3_2", "Levenberg: Dense solver (fixed blocksize)", "Dense", true, 3, 2)));
      factory->registerSolver(new DenseSolverCreator(OptimizationAlgorithmProperty("lm_dense6_3", "Levenberg: Dense solver (fixed blocksize)", "Dense", true, 6, 3)));
      factory->registerSolver(new DenseSolverCreator(OptimizationAlgorithmProperty("lm_dense7_3", "Levenberg: Dense solver (fixed blocksize)", "Dense", true, 7, 3)));
      initialized = true;
    }
  }

} // end namespace
