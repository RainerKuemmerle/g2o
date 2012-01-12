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

#include "solver_pcg.h"
#include "linear_solver_pcg.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/solver_factory.h"
#include "g2o/stuff/macros.h"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_PCG(s, p, l) \
  if (1) { \
      std::cerr << "# Using PCG poseDim " << p << " landMarkDim " << l << std::endl; \
      DIM_TO_SOLVER(p, l)::LinearSolverType* linearSolver = new LinearSolverPCG<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
      s = new DIM_TO_SOLVER(p, l)(linearSolver); \
  } else (void)0

namespace g2o {

  static OptimizationAlgorithm* createSolver(const std::string& fullSolverName)
  {
    g2o::Solver* s = 0;

    string methodName = fullSolverName.substr(0, 2);
    string solverName = fullSolverName.substr(3);

    if (solverName == "pcg") {
      ALLOC_PCG(s, -1, -1);
    }
    else if (solverName == "pcg3_2") {
      ALLOC_PCG(s, 3, 2);
    }
    else if (solverName == "pcg6_3") {
      ALLOC_PCG(s, 6, 3);
    }
    else if (solverName == "pcg7_3") {
      ALLOC_PCG(s, 7, 3);
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

  class PCGSolverCreator : public AbstractSolverCreator
  {
    public:
      PCGSolverCreator(const SolverProperty& p) : AbstractSolverCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };

  G2O_ATTRIBUTE_CONSTRUCTOR(init_solver_pcg)
  {
    solver_pcg::init();
  }

  namespace solver_pcg {
    void init()
    {
      static bool initialized = false;
      if (initialized)
        return;
      SolverFactory* factory = SolverFactory::instance();
      factory->registerSolver(new PCGSolverCreator(SolverProperty("gn_pcg", "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (variable blocksize)", "PCG", false, -1, -1)));
      factory->registerSolver(new PCGSolverCreator(SolverProperty("gn_pcg3_2", "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 3, 2)));
      factory->registerSolver(new PCGSolverCreator(SolverProperty("gn_pcg6_3", "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 6, 3)));
      factory->registerSolver(new PCGSolverCreator(SolverProperty("gn_pcg7_3", "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 7, 3)));
      factory->registerSolver(new PCGSolverCreator(SolverProperty("lm_pcg", "Levenberg: PCG solver using block-Jacobi pre-conditioner (variable blocksize)", "PCG", false, -1, -1)));
      factory->registerSolver(new PCGSolverCreator(SolverProperty("lm_pcg3_2", "Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 3, 2)));
      factory->registerSolver(new PCGSolverCreator(SolverProperty("lm_pcg6_3", "Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 6, 3)));
      factory->registerSolver(new PCGSolverCreator(SolverProperty("lm_pcg7_3", "Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 7, 3)));

      initialized = true;
    }
  }

}
