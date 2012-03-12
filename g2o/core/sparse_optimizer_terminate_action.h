#ifndef SPARSE_OPTIMIZER_TERMINATE_ACTION_H
#define SPARSE_OPTIMIZER_TERMINATE_ACTION_H

#include "hyper_graph_action.h"

namespace g2o {

  class G2O_CORE_API SparseOptimizerTerminateAction : public HyperGraphAction
  {
    public:
      SparseOptimizerTerminateAction();
      virtual HyperGraphAction* operator()(const HyperGraph* graph, Parameters* parameters = 0);

      double gainThreshold() const { return _gainThreshold;}
      void setGainThreshold(double gainThreshold);

    protected:
      double _gainThreshold;
      double _lastChi;
      bool _auxTerminateFlag;
  };

} // end namespace

#endif
