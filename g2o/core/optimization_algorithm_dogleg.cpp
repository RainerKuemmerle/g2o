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

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <utility>

#include "batch_stats.h"
#include "block_solver.h"
#include "g2o/core/optimization_algorithm.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/stuff/logger.h"
#include "g2o/stuff/misc.h"
#include "g2o/stuff/property.h"
#include "g2o/stuff/timeutil.h"
#include "solver.h"
#include "sparse_optimizer.h"

namespace g2o {

OptimizationAlgorithmDogleg::OptimizationAlgorithmDogleg(
    std::unique_ptr<BlockSolverBase> solver)
    : OptimizationAlgorithmWithHessian(*solver), m_solver_{std::move(solver)} {
  userDeltaInit_ = properties_.makeProperty<Property<double>>(
      "initialDelta", static_cast<double>(1e4));
  maxTrialsAfterFailure_ =
      properties_.makeProperty<Property<int>>("maxTrialsAfterFailure", 100);
  initialLambda_ = properties_.makeProperty<Property<double>>(
      "initialLambda", static_cast<double>(1e-7));
  lamdbaFactor_ =
      properties_.makeProperty<Property<double>>("lambdaFactor", 10.);
  delta_ = userDeltaInit_->value();
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
      G2O_WARN("Failure while building CCS structure");
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

  double t = get_monotonic_time();
  optimizer_->computeActiveErrors();
  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats) {
    globalStats->timeResiduals = get_monotonic_time() - t;
    t = get_monotonic_time();
  }

  const double currentChi = optimizer_->activeRobustChi2();

  solver_.buildSystem();
  if (globalStats) {
    globalStats->timeQuadraticForm = get_monotonic_time() - t;
  }

  VectorX::ConstMapType b(solver_.b(), solver_.vectorSize());

  // compute alpha
  auxVector_.setZero();
  blockSolver.multiplyHessian(auxVector_.data(), solver_.b());
  const double bNormSquared = b.squaredNorm();
  const double alpha = bNormSquared / auxVector_.dot(b);

  hsd_ = alpha * b;
  const double hsdNorm = hsd_.norm();
  double hgnNorm = -1.;

  bool solvedGaussNewton = false;
  bool goodStep = false;
  int& numTries = lastNumTries_;
  numTries = 0;
  do {
    ++numTries;

    if (!solvedGaussNewton) {
      const double minLambda = cst(1e-12);
      const double maxLambda = cst(1e3);
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
      const double c = hsd_.dot(auxVector_);
      const double bmaSquaredNorm = auxVector_.squaredNorm();
      double beta;
      if (c <= 0.)
        beta = (-c + sqrt(c * c + bmaSquaredNorm *
                                      (delta_ * delta_ - hsd_.squaredNorm()))) /
               bmaSquaredNorm;
      else {
        const double hsdSqrNorm = hsd_.squaredNorm();
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
    double linearGain = -1 * (auxVector_.dot(hdl_)) + 2 * (b.dot(hdl_));

    // apply the update and see what happens
    optimizer_->push();
    optimizer_->update(hdl_.data());
    optimizer_->computeActiveErrors();
    const double newChi = optimizer_->activeRobustChi2();
    const double nonLinearGain = currentChi - newChi;
    if (fabs(linearGain) < 1e-12) linearGain = cst(1e-12);
    const double rho = nonLinearGain / linearGain;
    if (rho > 0) {  // step is good and will be accepted
      optimizer_->discardTop();
      goodStep = true;
    } else {  // recover previous state
      optimizer_->pop();
    }

    // update trust region based on the step quality
    if (rho > 0.75)
      delta_ = std::max<double>(delta_, 3 * hdl_.norm());
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
