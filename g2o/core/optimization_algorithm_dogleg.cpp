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

#include "optimization_algorithm_dogleg.h"

#include <iostream>

#include "batch_stats.h"
#include "block_solver.h"
#include "g2o/stuff/timeutil.h"
#include "solver.h"
#include "sparse_optimizer.h"

namespace g2o {

OptimizationAlgorithmDogleg::OptimizationAlgorithmDogleg(
    std::unique_ptr<BlockSolverBase> solver)
    : OptimizationAlgorithmWithHessian(*solver), m_solver_{std::move(solver)} {
  userDeltaInit_ = properties_.makeProperty<Property<number_t>>(
      "initialDelta", static_cast<number_t>(1e4));
  maxTrialsAfterFailure_ =
      properties_.makeProperty<Property<int>>("maxTrialsAfterFailure", 100);
  initialLambda_ = properties_.makeProperty<Property<number_t>>(
      "initialLambda", static_cast<number_t>(1e-7));
  lamdbaFactor_ =
      properties_.makeProperty<Property<number_t>>("lambdaFactor", 10.);
  delta_ = userDeltaInit_->value();
  lastStep_ = kStepUndefined;
  wasPDInAllIterations_ = true;
  lastNumTries_ = 0;
  currentLambda_ = 0.;
}

OptimizationAlgorithmDogleg::~OptimizationAlgorithmDogleg() = default;

OptimizationAlgorithm::SolverResult OptimizationAlgorithmDogleg::solve(
    int iteration, bool online) {
  assert(optimizer_ && "optimizer_ not set");
  assert(solver_.optimizer() == optimizer_ &&
         "underlying linear solver operates on different graph");

  auto& blockSolver = static_cast<BlockSolverBase&>(solver_);

  if (iteration == 0 &&
      !online) {  // built up the CCS structure, here due to easy time measure
    const bool ok = solver_.buildStructure();
    if (!ok) {
      std::cerr << __PRETTY_FUNCTION__
                << ": Failure while building CCS structure" << std::endl;
      return OptimizationAlgorithm::kFail;
    }

    // init some members to the current size of the problem
    hsd_.resize(solver_.vectorSize());
    hdl_.resize(solver_.vectorSize());
    auxVector_.resize(solver_.vectorSize());
    delta_ = userDeltaInit_->value();
    currentLambda_ = initialLambda_->value();
    wasPDInAllIterations_ = true;
  }

  number_t t = get_monotonic_time();
  optimizer_->computeActiveErrors();
  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats) {
    globalStats->timeResiduals = get_monotonic_time() - t;
    t = get_monotonic_time();
  }

  const number_t currentChi = optimizer_->activeRobustChi2();

  solver_.buildSystem();
  if (globalStats) {
    globalStats->timeQuadraticForm = get_monotonic_time() - t;
  }

  VectorX::ConstMapType b(solver_.b(), solver_.vectorSize());

  // compute alpha
  auxVector_.setZero();
  blockSolver.multiplyHessian(auxVector_.data(), solver_.b());
  const number_t bNormSquared = b.squaredNorm();
  const number_t alpha = bNormSquared / auxVector_.dot(b);

  hsd_ = alpha * b;
  const number_t hsdNorm = hsd_.norm();
  number_t hgnNorm = -1.;

  bool solvedGaussNewton = false;
  bool goodStep = false;
  int& numTries = lastNumTries_;
  numTries = 0;
  do {
    ++numTries;

    if (!solvedGaussNewton) {
      const number_t minLambda = cst(1e-12);
      const number_t maxLambda = cst(1e3);
      solvedGaussNewton = true;
      // apply a damping factor to enforce positive definite Hessian, if the
      // matrix appeared to be not positive definite in at least one iteration
      // before. We apply a damping factor to obtain a PD matrix.
      bool solverOk = false;
      while (!solverOk) {
        if (!wasPDInAllIterations_)
          solver_.setLambda(currentLambda_,
                            true);  // add _currentLambda to the diagonal
        solverOk = solver_.solve();
        if (!wasPDInAllIterations_) solver_.restoreDiagonal();
        wasPDInAllIterations_ = wasPDInAllIterations_ && solverOk;
        if (!wasPDInAllIterations_) {
          // simple strategy to control the damping factor
          if (solverOk) {
            currentLambda_ =
                std::max(minLambda,
                         currentLambda_ / (cst(0.5) * lamdbaFactor_->value()));
          } else {
            currentLambda_ *= lamdbaFactor_->value();
            if (currentLambda_ > maxLambda) {
              currentLambda_ = maxLambda;
              return kFail;
            }
          }
        }
      }
      hgnNorm = VectorX::ConstMapType(solver_.x(), solver_.vectorSize()).norm();
    }

    VectorX::ConstMapType hgn(solver_.x(), solver_.vectorSize());
    assert(hgnNorm >= 0. && "Norm of the GN step is not computed");

    if (hgnNorm < delta_) {
      hdl_ = hgn;
      lastStep_ = kStepGn;
    } else if (hsdNorm > delta_) {
      hdl_ = delta_ / hsdNorm * hsd_;
      lastStep_ = kStepSd;
    } else {
      auxVector_ = hgn - hsd_;  // b - a
      const number_t c = hsd_.dot(auxVector_);
      const number_t bmaSquaredNorm = auxVector_.squaredNorm();
      number_t beta;
      if (c <= 0.)
        beta = (-c + sqrt(c * c + bmaSquaredNorm *
                                      (delta_ * delta_ - hsd_.squaredNorm()))) /
               bmaSquaredNorm;
      else {
        const number_t hsdSqrNorm = hsd_.squaredNorm();
        beta =
            (delta_ * delta_ - hsdSqrNorm) /
            (c + sqrt(c * c + bmaSquaredNorm * (delta_ * delta_ - hsdSqrNorm)));
      }
      assert(beta > 0. && beta < 1 && "Error while computing beta");
      hdl_ = hsd_ + beta * (hgn - hsd_);
      lastStep_ = kStepDl;
      assert(hdl_.norm() < delta_ + 1e-5 &&
             "Computed step does not correspond to the trust region");
    }

    // compute the linear gain
    auxVector_.setZero();
    blockSolver.multiplyHessian(auxVector_.data(), hdl_.data());
    number_t linearGain = -1 * (auxVector_.dot(hdl_)) + 2 * (b.dot(hdl_));

    // apply the update and see what happens
    optimizer_->push();
    optimizer_->update(hdl_.data());
    optimizer_->computeActiveErrors();
    const number_t newChi = optimizer_->activeRobustChi2();
    const number_t nonLinearGain = currentChi - newChi;
    if (fabs(linearGain) < 1e-12) linearGain = cst(1e-12);
    const number_t rho = nonLinearGain / linearGain;
    // cerr << PVAR(nonLinearGain) << " " << PVAR(linearGain) << " " <<
    // PVAR(rho) << endl;
    if (rho > 0) {  // step is good and will be accepted
      optimizer_->discardTop();
      goodStep = true;
    } else {  // recover previous state
      optimizer_->pop();
    }

    // update trust region based on the step quality
    if (rho > 0.75)
      delta_ = std::max<number_t>(delta_, 3 * hdl_.norm());
    else if (rho < 0.25)
      delta_ *= 0.5;
  } while (!goodStep && numTries < maxTrialsAfterFailure_->value());
  if (numTries == maxTrialsAfterFailure_->value() || !goodStep)
    return kTerminate;
  return kOk;
}

void OptimizationAlgorithmDogleg::printVerbose(std::ostream& os) const {
  os << "\t Delta= " << delta_ << "\t step= " << stepType2Str(lastStep_)
     << "\t tries= " << lastNumTries_;
  if (!wasPDInAllIterations_) os << "\t lambda= " << currentLambda_;
}

const char* OptimizationAlgorithmDogleg::stepType2Str(int stepType) {
  switch (stepType) {
    case kStepSd:
      return "Descent";
    case kStepGn:
      return "GN";
    case kStepDl:
      return "Dogleg";
    default:
      return "Undefined";
  }
}

}  // namespace g2o
