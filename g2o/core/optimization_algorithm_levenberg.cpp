
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

#include "optimization_algorithm_levenberg.h"

#include <iostream>

#include "batch_stats.h"
#include "g2o/stuff/timeutil.h"
#include "solver.h"
#include "sparse_optimizer.h"

namespace g2o {

OptimizationAlgorithmLevenberg::OptimizationAlgorithmLevenberg(
    std::unique_ptr<Solver> solver)
    : OptimizationAlgorithmWithHessian(*solver), m_solver_{std::move(solver)} {
  userLambdaInit_ =
      properties_.makeProperty<Property<number_t> >("initialLambda", 0.);
  maxTrialsAfterFailure_ =
      properties_.makeProperty<Property<int> >("maxTrialsAfterFailure", 10);
}

OptimizationAlgorithm::SolverResult OptimizationAlgorithmLevenberg::solve(
    int iteration, bool online) {
  assert(optimizer_ && "optimizer_ not set");
  assert(solver_.optimizer() == optimizer_ &&
         "underlying linear solver operates on different graph");

  if (iteration == 0 &&
      !online) {  // built up the CCS structure, here due to easy time measure
    const bool ok = solver_.buildStructure();
    if (!ok) {
      std::cerr << __PRETTY_FUNCTION__
                << ": Failure while building CCS structure" << std::endl;
      return OptimizationAlgorithm::kFail;
    }
  }

  number_t t = get_monotonic_time();
  optimizer_->computeActiveErrors();
  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats) {
    globalStats->timeResiduals = get_monotonic_time() - t;
    t = get_monotonic_time();
  }

  number_t currentChi = optimizer_->activeRobustChi2();

  solver_.buildSystem();
  if (globalStats) {
    globalStats->timeQuadraticForm = get_monotonic_time() - t;
  }

  // core part of the Levenbarg algorithm
  if (iteration == 0) {
    currentLambda_ = computeLambdaInit();
    ni_ = 2;
  }

  number_t rho = 0;
  int& qmax = levenbergIterations_;
  qmax = 0;
  do {
    optimizer_->push();
    if (globalStats) {
      globalStats->levenbergIterations++;
      t = get_monotonic_time();
    }
    // update the diagonal of the system matrix
    solver_.setLambda(currentLambda_, true);
    const bool ok2 = solver_.solve();
    if (globalStats) {
      globalStats->timeLinearSolution += get_monotonic_time() - t;
      t = get_monotonic_time();
    }
    optimizer_->update(solver_.x());
    if (globalStats) {
      globalStats->timeUpdate = get_monotonic_time() - t;
    }

    // restore the diagonal
    solver_.restoreDiagonal();

    optimizer_->computeActiveErrors();
    const number_t tempChi = ok2 ? optimizer_->activeRobustChi2()
                                 : std::numeric_limits<number_t>::max();

    rho = (currentChi - tempChi);
    number_t scale = computeScale();
    scale += cst(1e-3);  // make sure it's non-zero :)
    rho /= scale;

    if (rho > 0 && g2o_isfinite(tempChi)) {  // last step was good
      number_t alpha = 1. - pow((2 * rho - 1), 3);
      // crop lambda between minimum and maximum factors
      alpha = (std::min)(alpha, goodStepUpperScale_);
      const number_t scaleFactor = (std::max)(goodStepLowerScale_, alpha);
      currentLambda_ *= scaleFactor;
      ni_ = 2;
      currentChi = tempChi;
      optimizer_->discardTop();
    } else {
      currentLambda_ *= ni_;
      ni_ *= 2;
      optimizer_->pop();  // restore the last state before trying to optimize
      if (!g2o_isfinite(currentLambda_)) break;
    }
    qmax++;
  } while (rho < 0 && qmax < maxTrialsAfterFailure_->value() &&
           !optimizer_->terminate());

  if (qmax == maxTrialsAfterFailure_->value() || rho == 0 ||
      !g2o_isfinite(currentLambda_))
    return kTerminate;
  return kOk;
}

number_t OptimizationAlgorithmLevenberg::computeLambdaInit() const {
  if (userLambdaInit_->value() > 0) return userLambdaInit_->value();
  number_t maxDiagonal = 0;
  for (auto* v : optimizer_->indexMapping()) {
    assert(v);
    MatrixN<Eigen::Dynamic>::MapType hessian = v->hessianMap();
    maxDiagonal =
        std::max(hessian.diagonal().cwiseAbs().maxCoeff(), maxDiagonal);
  }
  return tau_ * maxDiagonal;
}

number_t OptimizationAlgorithmLevenberg::computeScale() const {
  number_t scale = 0;
  for (size_t j = 0; j < solver_.vectorSize(); j++) {
    scale +=
        solver_.x()[j] * (currentLambda_ * solver_.x()[j] + solver_.b()[j]);
  }
  return scale;
}

void OptimizationAlgorithmLevenberg::setMaxTrialsAfterFailure(int max_trials) {
  maxTrialsAfterFailure_->setValue(max_trials);
}

void OptimizationAlgorithmLevenberg::setUserLambdaInit(number_t lambda) {
  userLambdaInit_->setValue(lambda);
}

void OptimizationAlgorithmLevenberg::printVerbose(std::ostream& os) const {
  os << "\t schur= " << solver_.schur()
     << "\t lambda= " << FIXED(currentLambda_)
     << "\t levenbergIter= " << levenbergIterations_;
}

}  // namespace g2o
