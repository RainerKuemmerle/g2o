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

#include "optimization_algorithm_gauss_newton.h"

#include <iostream>

#include "batch_stats.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/timeutil.h"
#include "solver.h"
#include "sparse_optimizer.h"

namespace g2o {

OptimizationAlgorithmGaussNewton::OptimizationAlgorithmGaussNewton(
    std::unique_ptr<Solver> solver)
    : OptimizationAlgorithmWithHessian(*solver), m_solver_{std::move(solver)} {}

OptimizationAlgorithm::SolverResult OptimizationAlgorithmGaussNewton::solve(
    int iteration, bool online) {
  assert(solver_.optimizer() == optimizer_ &&
         "underlying linear solver operates on different graph");
  bool ok = true;

  // here so that correct component for max-mixtures can be computed before the
  // build structure
  number_t t = get_monotonic_time();
  optimizer_->computeActiveErrors();
  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats) {
    globalStats->timeResiduals = get_monotonic_time() - t;
  }

  if (iteration == 0 &&
      !online) {  // built up the CCS structure, here due to easy time measure
    ok = solver_.buildStructure();
    if (!ok) {
      std::cerr << __PRETTY_FUNCTION__
                << ": Failure while building CCS structure" << std::endl;
      return OptimizationAlgorithm::kFail;
    }
  }

  t = get_monotonic_time();
  solver_.buildSystem();
  if (globalStats) {
    globalStats->timeQuadraticForm = get_monotonic_time() - t;
    t = get_monotonic_time();
  }

  ok = solver_.solve();
  if (globalStats) {
    globalStats->timeLinearSolution = get_monotonic_time() - t;
    t = get_monotonic_time();
  }

  optimizer_->update(solver_.x());
  if (globalStats) {
    globalStats->timeUpdate = get_monotonic_time() - t;
  }
  if (ok) return kOk;
  return kFail;
}

void OptimizationAlgorithmGaussNewton::printVerbose(std::ostream& os) const {
  os << "\t schur= " << solver_.schur();
}

}  // namespace g2o
