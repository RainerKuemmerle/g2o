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

#include "linear_solver_cholmod.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_factory.h"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/stuff/macros.h"

//#define ADD_SCALAR_ORDERING

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_CHOLMOD(s, p, l, blockorder) \
  if (1) { \
    std::cerr << "# Using CHOLMOD poseDim " << p << " landMarkDim " << l << " blockordering " << blockorder << std::endl; \
    LinearSolverCholmod < DIM_TO_SOLVER(p, l)::PoseMatrixType >* linearSolver = new LinearSolverCholmod<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
    linearSolver->setBlockOrdering(blockorder); \
    s = new DIM_TO_SOLVER(p, l)(linearSolver); \
  } else (void)0

using namespace std;

namespace g2o {

  static OptimizationAlgorithm* createSolver(const std::string& fullSolverName)
  {
    g2o::Solver* s = 0;

    string methodName = fullSolverName.substr(0, 2);
    string solverName = fullSolverName.substr(3);

    if (solverName == "var_cholmod") {
      ALLOC_CHOLMOD(s, -1, -1, false);
    }
    else if (solverName == "fix3_2_cholmod") {
      ALLOC_CHOLMOD(s, 3, 2, true);
    }
    else if (solverName == "fix6_3_cholmod") {
      ALLOC_CHOLMOD(s, 6, 3, true);
    }
    else if (solverName == "fix7_3_cholmod") {
      ALLOC_CHOLMOD(s, 7, 3, true);
    }
#ifdef ADD_SCALAR_ORDERING
    else if (solverName == "fix3_2_cholmod_scalar") {
      ALLOC_CHOLMOD(s, 3, 2, false);
    }
    else if (solverName == "fix6_3_cholmod_scalar") {
      ALLOC_CHOLMOD(s, 6, 3, false);
    }
    else if (solverName == "fix7_3_cholmod_scalar") {
      ALLOC_CHOLMOD(s, 7, 3, false);
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
    else {
      delete s;
    }

    return snl;
  }

  class CholmodSolverCreator : public AbstractOptimizationAlgorithmCreator
  {
    public:
      CholmodSolverCreator(const OptimizationAlgorithmProperty& p) : AbstractOptimizationAlgorithmCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };

  G2O_REGISTER_OPTIMIZATION_LIBRARY(cholmod);

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_var_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_var_cholmod", "Gauss-Newton: Cholesky solver using CHOLMOD (variable blocksize)", "CHOLMOD", false, Eigen::Dynamic, Eigen::Dynamic)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix3_2_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix3_2_cholmod", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix6_3_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix6_3_cholmod", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix7_3_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix7_3_cholmod", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 7, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_var_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_var_cholmod", "Levenberg: Cholesky solver using CHOLMOD (variable blocksize)", "CHOLMOD", false, Eigen::Dynamic, Eigen::Dynamic)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix3_2_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix3_2_cholmod", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix6_3_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix6_3_cholmod", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix7_3_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix7_3_cholmod", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 7, 3)));

#ifdef ADD_SCALAR_ORDERING
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix3_2_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix3_2_cholmod_scalar", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix6_3_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix6_3_cholmod_scalar", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix7_3_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix7_3_cholmod_scalar", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 7, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix3_2_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix3_2_cholmod_scalar", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix6_3_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix6_3_cholmod_scalar", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix7_3_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix7_3_cholmod_scalar", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 7, 3)));
#endif

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(dl_var_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("dl_var_cholmod", "Dogleg: Cholesky solver using CHOLMOD (variable blocksize)", "CHOLMOD", false, Eigen::Dynamic, Eigen::Dynamic)));
}
