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

#include <Eigen/Cholesky>
#include <memory>
#include <string>
#include <vector>

#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "structure_only_solver.h"  // IWYU pragma: keep

namespace g2o {

namespace {
/**
 * helper function for allocating
 */
std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  if (fullSolverName == "structure_only_2") {
    return std::unique_ptr<OptimizationAlgorithm>(new StructureOnlySolver<2>);
  }
  if (fullSolverName == "structure_only_3") {
    return std::unique_ptr<OptimizationAlgorithm>(new StructureOnlySolver<3>);
  }
  return nullptr;
}
}  // namespace

class StructureOnlyCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  explicit StructureOnlyCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(structure_only);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    structure_only_2,
    std::make_shared<StructureOnlyCreator>(OptimizationAlgorithmProperty(
        "structure_only_2", "Optimize the landmark poses (2D)", "Eigen", true,
        3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    structure_only_3,
    std::make_shared<StructureOnlyCreator>(OptimizationAlgorithmProperty(
        "structure_only_3", "Optimize the landmark poses (3D)", "Eigen", true,
        6, 3)));

}  // namespace g2o
