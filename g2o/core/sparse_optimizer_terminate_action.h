#ifndef SPARSE_OPTIMIZER_TERMINATE_ACTION_H
#define SPARSE_OPTIMIZER_TERMINATE_ACTION_H

#include "g2o_core_api.h"
#include "hyper_graph_action.h"

namespace g2o {

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
  };

} // end namespace

#endif
