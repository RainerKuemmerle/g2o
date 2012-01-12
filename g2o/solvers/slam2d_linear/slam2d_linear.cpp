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

#include "slam2d_linear.h"
#include "solver_slam2d_linear.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/solver_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm.h"

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
    if (fullSolverName != "2dlinear")
      return 0;

    g2o::Solver* s = 0;
    ALLOC_CSPARSE(s, 3, 2, true);
    OptimizationAlgorithm* snl = 0;
    snl = new SolverSLAM2DLinear(s);

    return snl;
  }

  class SLAM2DLinearSolverCreator : public AbstractSolverCreator
  {
    public:
      SLAM2DLinearSolverCreator(const SolverProperty& p) : AbstractSolverCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };

  G2O_ATTRIBUTE_CONSTRUCTOR(init_solver_csparse)
  {
    solver_slam2d_linear::init();
  }

  namespace solver_slam2d_linear {
    void init()
    {
      static bool initialized = false;
      if (initialized)
        return;
      SolverFactory* factory = SolverFactory::instance();
      factory->registerSolver(new SLAM2DLinearSolverCreator(SolverProperty("2dlinear", "Solve Orientation + Gauss-Newton: Works only on 2D pose graphs!!", "CSparse", false, 3, 3)));
      initialized = 1;
    }
  }

} // end namespace
