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

#ifndef SPARSE_OPTIMIZER_TERMINATE_ACTION_H
#define SPARSE_OPTIMIZER_TERMINATE_ACTION_H

#include "g2o_core_api.h"
#include "hyper_graph_action.h"

namespace g2o {

  class SparseOptimizer;

  /**
   * \brief stop iterating based on the gain which is (oldChi - currentChi) / currentChi.
   *
   * stop iterating based on the gain which is (oldChi - currentChi) / currentChi.
   * If the gain is larger than zero and below the threshold, then the optimizer is stopped.
   * Typically usage of this action includes adding it as a postIteration action, by calling
   * addPostIterationAction on a sparse optimizer.
   */
  class G2O_CORE_API SparseOptimizerTerminateAction : public HyperGraphAction
  {
    public:
      SparseOptimizerTerminateAction();
      virtual HyperGraphAction* operator()(const HyperGraph* graph, Parameters* parameters = 0);

      double gainThreshold() const { return _gainThreshold;}
      void setGainThreshold(double gainThreshold);

      int maxIterations() const { return _maxIterations;}
      void setMaxIterations(int maxit);

    protected:
      double _gainThreshold;
      double _lastChi;
      bool _auxTerminateFlag;
      int _maxIterations;

      void setOptimizerStopFlag(const SparseOptimizer* optimizer, bool stop);
  };

} // end namespace

#endif
