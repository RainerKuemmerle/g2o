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

#include "solver_csparse.h"
#include "linear_solver_csparse.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/solver_factory.h"
#include "g2o/core/sparse_optimizer.h"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/stuff/macros.h"

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_CSPARSE(s, p, l, blockorder) \
  if (1) { \
    std::cerr << "# Using CSparse poseDim " << p << " landMarkDim " << l << " blockordering " << blockorder << std::endl; \
    LinearSolverCSparse< DIM_TO_SOLVER(p, l)::PoseMatrixType >* linearSolver = new LinearSolverCSparse<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
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

    if (solverName == "var") {
      ALLOC_CSPARSE(s, -1, -1, false);
    }
    else if (solverName == "fix3_2") {
      ALLOC_CSPARSE(s, 3, 2, true);
    }
    else if (solverName == "fix6_3") {
      ALLOC_CSPARSE(s, 6, 3, true);
    }
    else if (solverName == "fix7_3") {
      ALLOC_CSPARSE(s, 7, 3, true);
    }
    else if (solverName == "fix3_2_scalar") {
      ALLOC_CSPARSE(s, 3, 2, false);
    }
    else if (solverName == "fix6_3_scalar") {
      ALLOC_CSPARSE(s, 6, 3, false);
    }
    else if (solverName == "fix7_3_scalar") {
      ALLOC_CSPARSE(s, 7, 3, false);
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

  class CSparseSolverCreator : public AbstractSolverCreator
  {
    public:
      CSparseSolverCreator(const SolverProperty& p) : AbstractSolverCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };


  namespace solver_csparse {
    void init()
    {
      static bool initialized = false;
      if (initialized)
        return;
      SolverFactory* factory = SolverFactory::instance();
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("gn_var", "Gauss-Newton: Cholesky solver using CSparse (variable blocksize)", "CSparse", false, -1, -1)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("gn_fix3_2", "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 3, 2)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("gn_fix6_3", "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 6, 3)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("gn_fix7_3", "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 7, 3)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("gn_fix3_2_scalar", "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 3, 2)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("gn_fix6_3_scalar", "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 6, 3)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("gn_fix7_3_scalar", "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 7, 3)));

      factory->registerSolver(new CSparseSolverCreator(SolverProperty("lm_var", "Levenberg: Cholesky solver using CSparse (variable blocksize)", "CSparse", false, -1, -1)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("lm_fix3_2", "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 3, 2)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("lm_fix6_3", "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 6, 3)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("lm_fix7_3", "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 7, 3)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("lm_fix3_2_scalar", "Levenberg: Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 3, 2)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("lm_fix6_3_scalar", "Levenberg: Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 6, 3)));
      factory->registerSolver(new CSparseSolverCreator(SolverProperty("lm_fix7_3_scalar", "Levenberg: Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 7, 3)));

      initialized = true;
    }
  }

  G2O_ATTRIBUTE_CONSTRUCTOR(init_solver_csparse)
  {
    solver_csparse::init();
  }

} // end namespace
