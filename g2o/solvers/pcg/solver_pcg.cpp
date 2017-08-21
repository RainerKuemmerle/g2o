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

#include "linear_solver_pcg.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/stuff/macros.h"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

using namespace std;

namespace g2o
{
  namespace
  {
    template<int p, int l>
    std::unique_ptr<g2o::Solver> AllocateSolver()
    {
      std::cerr << "# Using PCG poseDim " << p << " landMarkDim " << l << std::endl;

      return g2o::make_unique<BlockSolverPL<p, l>>(
        g2o::make_unique<LinearSolverPCG<typename BlockSolverPL<p, l>::PoseMatrixType>>());
    }
  }

  static OptimizationAlgorithm* createSolver(const std::string& fullSolverName)
  {
    static const std::map<std::string, std::function<std::unique_ptr<g2o::Solver>()>> solver_factories{
        { "pcg", &AllocateSolver<-1, -1> },
        { "pcg3_2", &AllocateSolver<3, 2> },
        { "pcg6_3", &AllocateSolver<6, 3> },
        { "pcg7_3", &AllocateSolver<7, 3> },
    };

    string solverName = fullSolverName.substr(3);
    auto solverf = solver_factories.find(solverName);
    if (solverf == solver_factories.end())
      return nullptr;

    string methodName = fullSolverName.substr(0, 2);

    if (methodName == "gn") {
      return new OptimizationAlgorithmGaussNewton(solverf->second());
    }
    else if (methodName == "lm") {
      return new OptimizationAlgorithmLevenberg(solverf->second());
    }

    return nullptr;
  }

  class PCGSolverCreator : public AbstractOptimizationAlgorithmCreator
  {
    public:
      PCGSolverCreator(const OptimizationAlgorithmProperty& p) : AbstractOptimizationAlgorithmCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };

  G2O_REGISTER_OPTIMIZATION_LIBRARY(pcg);

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_pcg, new PCGSolverCreator(OptimizationAlgorithmProperty("gn_pcg", "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (variable blocksize)", "PCG", false, Eigen::Dynamic, Eigen::Dynamic)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_pcg3_2, new PCGSolverCreator(OptimizationAlgorithmProperty("gn_pcg3_2", "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_pcg6_3, new PCGSolverCreator(OptimizationAlgorithmProperty("gn_pcg6_3", "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_pcg7_3, new PCGSolverCreator(OptimizationAlgorithmProperty("gn_pcg7_3", "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 7, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_pcg, new PCGSolverCreator(OptimizationAlgorithmProperty("lm_pcg", "Levenberg: PCG solver using block-Jacobi pre-conditioner (variable blocksize)", "PCG", false, Eigen::Dynamic, Eigen::Dynamic)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_pcg3_2, new PCGSolverCreator(OptimizationAlgorithmProperty("lm_pcg3_2", "Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_pcg6_3, new PCGSolverCreator(OptimizationAlgorithmProperty("lm_pcg6_3", "Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_pcg7_3, new PCGSolverCreator(OptimizationAlgorithmProperty("lm_pcg7_3", "Levenberg: PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 7, 3)));
}
