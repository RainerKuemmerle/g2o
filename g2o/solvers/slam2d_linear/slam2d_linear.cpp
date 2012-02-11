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

#include "solver_slam2d_linear.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
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

  class SLAM2DLinearSolverCreator : public AbstractOptimizationAlgorithmCreator
  {
    public:
      SLAM2DLinearSolverCreator(const OptimizationAlgorithmProperty& p) : AbstractOptimizationAlgorithmCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };

  G2O_REGISTER_OPTIMIZATION_LIBRARY(slam2d_linear);

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(2dlinear, new SLAM2DLinearSolverCreator(OptimizationAlgorithmProperty("2dlinear", "Solve Orientation + Gauss-Newton: Works only on 2D pose graphs!!", "CSparse", false, 3, 3)));

} // end namespace
