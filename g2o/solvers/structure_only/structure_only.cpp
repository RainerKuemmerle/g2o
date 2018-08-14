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

#include "structure_only_solver.h"

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/stuff/macros.h"

namespace g2o {

  /**
   * helper function for allocating
   */
  static OptimizationAlgorithm* createSolver(const std::string& fullSolverName)
  {
    if (fullSolverName == "structure_only_2") {
      OptimizationAlgorithm* optimizationAlgo = new StructureOnlySolver<2>;
      return optimizationAlgo;
    }
    else if (fullSolverName == "structure_only_3") {
      OptimizationAlgorithm* optimizationAlgo = new StructureOnlySolver<3>;
      return optimizationAlgo;
    }
    else
      return nullptr;
  }

  class StructureOnlyCreator : public AbstractOptimizationAlgorithmCreator
  {
    public:
      StructureOnlyCreator(const OptimizationAlgorithmProperty& p) : AbstractOptimizationAlgorithmCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };

  G2O_REGISTER_OPTIMIZATION_LIBRARY(structure_only);

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(structure_only_2, new StructureOnlyCreator(OptimizationAlgorithmProperty("structure_only_2", "Optimize the landmark poses (2D)", "Eigen", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(structure_only_3, new StructureOnlyCreator(OptimizationAlgorithmProperty("structure_only_3", "Optimize the landmark poses (3D)", "Eigen", true, 6, 3)));

} // end namespace
