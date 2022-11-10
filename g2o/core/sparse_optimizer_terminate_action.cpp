// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "sparse_optimizer_terminate_action.h"

#include <limits>

#include "sparse_optimizer.h"

namespace g2o {

SparseOptimizerTerminateAction::SparseOptimizerTerminateAction()
    : HyperGraphAction(),
      gainThreshold_(cst(1e-6)),
      maxIterations_(std::numeric_limits<int>::max()) {}

void SparseOptimizerTerminateAction::setGainThreshold(number_t gainThreshold) {
  gainThreshold_ = gainThreshold;
}

bool SparseOptimizerTerminateAction::operator()(
    const HyperGraph& graph, const std::shared_ptr<Parameters>& parameters) {
  assert(dynamic_cast<const SparseOptimizer*>(&graph) &&
         "graph is not a SparseOptimizer");
  assert(
      dynamic_cast<HyperGraphAction::ParametersIteration*>(parameters.get()) &&
      "error casting parameters");

  const auto* optimizer = static_cast<const SparseOptimizer*>(&graph);
  auto params = std::static_pointer_cast<HyperGraphAction::ParametersIteration>(
      parameters);

  const_cast<SparseOptimizer*>(optimizer)->computeActiveErrors();
  if (params->iteration < 0) {
    // let the optimizer run for at least one iteration
    // Hence, we reset the stop flag
    setOptimizerStopFlag(optimizer, false);
  } else if (params->iteration == 0) {
    // first iteration, just store the chi2 value
    lastChi_ = optimizer->activeRobustChi2();
  } else {
    // compute the gain and stop the optimizer in case the
    // gain is below the threshold or we reached the max
    // number of iterations
    bool stopOptimizer = false;
    if (params->iteration < maxIterations_) {
      const number_t currentChi = optimizer->activeRobustChi2();
      const number_t gain = (lastChi_ - currentChi) / currentChi;
      lastChi_ = currentChi;
      stopOptimizer = gain >= 0 && gain < gainThreshold_;
    } else {
      stopOptimizer = true;
    }
    // tell the optimizer to stop
    setOptimizerStopFlag(optimizer, stopOptimizer);
  }
  return true;
}

void SparseOptimizerTerminateAction::setMaxIterations(int maxit) {
  maxIterations_ = maxit;
}

void SparseOptimizerTerminateAction::setOptimizerStopFlag(
    const SparseOptimizer* optimizer, bool stop) {
  if (optimizer->forceStopFlag()) {
    *(optimizer->forceStopFlag()) = stop;
  } else {
    auxTerminateFlag_ = stop;
    const_cast<SparseOptimizer*>(optimizer)->setForceStopFlag(
        &auxTerminateFlag_);
  }
}

}  // namespace g2o
