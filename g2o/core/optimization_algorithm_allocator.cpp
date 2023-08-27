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

#include "optimization_algorithm_allocator.h"

#include <memory>

#include "block_solver.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "optimization_algorithm.h"

namespace g2o {

std::unique_ptr<OptimizationAlgorithm> OptimizationAlgorithmAllocator::allocate(
    std::string_view fullSolverName, const AllocateMap& allocate_map) {
  const std::string solverName(fullSolverName.substr(3));
  auto solver_allocator = allocate_map.find(solverName);
  if (solver_allocator == allocate_map.end()) return nullptr;

  const std::string_view methodName = fullSolverName.substr(0, 2);

  if (methodName == "gn") {
    return std::unique_ptr<OptimizationAlgorithm>(
        new OptimizationAlgorithmGaussNewton(solver_allocator->second()));
  }
  if (methodName == "lm") {
    return std::unique_ptr<OptimizationAlgorithm>(
        new OptimizationAlgorithmLevenberg(solver_allocator->second()));
  }
  if (methodName == "dl") {
    return std::unique_ptr<OptimizationAlgorithm>(
        new OptimizationAlgorithmDogleg(solver_allocator->second()));
  }

  return nullptr;
}

std::unique_ptr<OptimizationAlgorithm> OptimizationAlgorithmAllocator::allocate(
    std::string_view fullSolverName, const AllocateSolverMap& allocate_map) {
  const std::string solverName(fullSolverName.substr(3));
  auto solver_allocator = allocate_map.find(solverName);
  if (solver_allocator == allocate_map.end()) return nullptr;

  const std::string_view methodName = fullSolverName.substr(0, 2);

  if (methodName == "gn") {
    return std::unique_ptr<OptimizationAlgorithm>(
        new OptimizationAlgorithmGaussNewton(solver_allocator->second()));
  }
  if (methodName == "lm") {
    return std::unique_ptr<OptimizationAlgorithm>(
        new OptimizationAlgorithmLevenberg(solver_allocator->second()));
  }
  return nullptr;
}

}  // namespace g2o
