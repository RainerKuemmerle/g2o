#include "sparse_optimizer_terminate_action.h"

#include "sparse_optimizer.h"

#include <limits>

namespace g2o {

  SparseOptimizerTerminateAction::SparseOptimizerTerminateAction() :
    HyperGraphAction(),
    _gainThreshold(1e-6), _lastChi(0.), _auxTerminateFlag(false),
    _maxIterations(std::numeric_limits<int>::max())
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
      bool stopOptimizer = false;
      if (params->iteration < _maxIterations) {
        double currentChi = optimizer->activeChi2();
        double gain = (_lastChi - currentChi) / currentChi;
        _lastChi = currentChi;
        if (gain >= 0 && gain < _gainThreshold)
          stopOptimizer = true;
      } else {
        stopOptimizer = true;
      }
      if (stopOptimizer) { // tell the optimizer to stop
        if (optimizer->forceStopFlag()) {
          *(optimizer->forceStopFlag()) = true;
        } else {
          _auxTerminateFlag = true;
          const_cast<SparseOptimizer*>(optimizer)->setForceStopFlag(&_auxTerminateFlag);
        }
      }
    }
    return this;
  }

  void SparseOptimizerTerminateAction::setMaxIterations(int maxit)
  {
    _maxIterations = maxit;
  }

} // end namespace
