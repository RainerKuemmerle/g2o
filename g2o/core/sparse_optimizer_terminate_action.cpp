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
    if (params->iteration < 0)
    {
      // let the optimizer run for at least one iteration
      // Hence, we reset the stop flag
      setOptimizerStopFlag(optimizer, false);
    } else if (params->iteration == 0) {
      // first iteration, just store the chi2 value
      _lastChi = optimizer->activeRobustChi2();
    } else {
      // compute the gain and stop the optimizer in case the
      // gain is below the threshold or we reached the max
      // number of iterations
      bool stopOptimizer = false;
      if (params->iteration < _maxIterations) {
        double currentChi = optimizer->activeRobustChi2();
        double gain = (_lastChi - currentChi) / currentChi;
        _lastChi = currentChi;
        if (gain >= 0 && gain < _gainThreshold)
          stopOptimizer = true;
      } else {
        stopOptimizer = true;
      }
      if (stopOptimizer) { // tell the optimizer to stop
        setOptimizerStopFlag(optimizer, true);
      }
    }
    return this;
  }

  void SparseOptimizerTerminateAction::setMaxIterations(int maxit)
  {
    _maxIterations = maxit;
  }

  void SparseOptimizerTerminateAction::setOptimizerStopFlag(const SparseOptimizer* optimizer, bool stop)
  {
    if (optimizer->forceStopFlag()) {
      *(optimizer->forceStopFlag()) = stop;
    } else {
      _auxTerminateFlag = stop;
      const_cast<SparseOptimizer*>(optimizer)->setForceStopFlag(&_auxTerminateFlag);
    }
  }

} // end namespace
