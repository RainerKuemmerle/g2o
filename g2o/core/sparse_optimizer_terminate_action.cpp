#include "sparse_optimizer_terminate_action.h"

#include "sparse_optimizer.h"

namespace g2o {

  SparseOptimizerTerminateAction::SparseOptimizerTerminateAction() :
    HyperGraphAction(),
    _gainThreshold(1e-6), _lastChi(0.), _auxTerminateFlag(false)
  {
  }

  void SparseOptimizerTerminateAction::setGainThreshold(double gainThreshold)
  {
    _gainThreshold = gainThreshold;
  }

  HyperGraphAction* SparseOptimizerTerminateAction::operator()(const HyperGraph* graph, Parameters* parameters)
  {
    assert(dynamic_cast<const SparseOptimizer*>(graph) && "graph is not a SparseOptimizer");
    assert(dynamic_cast<HyperGraphAction::ParametersIteration*>(parameters) && "error casting parameters");

    const SparseOptimizer* optimizer = static_cast<const SparseOptimizer*>(graph);
    HyperGraphAction::ParametersIteration* params = static_cast<HyperGraphAction::ParametersIteration*>(parameters);

    const_cast<SparseOptimizer*>(optimizer)->computeActiveErrors();
    if (params->iteration == 0) {
      _lastChi = optimizer->activeChi2();
    } else {
      double currentChi = optimizer->activeChi2();
      double gain = (_lastChi - currentChi) / currentChi;
      if (gain >= 0 && gain < _gainThreshold) {
        // tell the optimizer to stop
        if (optimizer->forceStopFlag()) {
          *(optimizer->forceStopFlag()) = true;
        } else {
          _auxTerminateFlag = true;
          const_cast<SparseOptimizer*>(optimizer)->setForceStopFlag(&_auxTerminateFlag);
        }
      }
      _lastChi = currentChi;
    }
    return this;
  }

} // end namespace
