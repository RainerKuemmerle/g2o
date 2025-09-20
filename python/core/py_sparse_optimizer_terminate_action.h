#pragma once

#include <g2o/core/sparse_optimizer_terminate_action.h>

#include "g2opy.h"

namespace g2o {

inline void delcareSparseOptimizerTerminateAction(py::module& m) {
  py::classh<SparseOptimizerTerminateAction, HyperGraphAction>(
      m, "SparseOptimizerTerminateAction")
      .def(py::init<>())
      .def("__call__", &SparseOptimizerTerminateAction::operator())
      .def("gain_threshold", &SparseOptimizerTerminateAction::gainThreshold)
      .def("set_gain_threshold",
           &SparseOptimizerTerminateAction::setGainThreshold)
      .def("max_iterations", &SparseOptimizerTerminateAction::maxIterations)
      .def("set_max_iterations",
           &SparseOptimizerTerminateAction::setMaxIterations);
}

}  // namespace g2o
