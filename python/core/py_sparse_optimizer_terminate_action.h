#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void delcareSparseOptimizerTerminateAction(py::module& m) {
  py::class_<SparseOptimizerTerminateAction, HyperGraphAction>(m, "SparseOptimizerTerminateAction")
      .def(py::init<>())
      .def("__call__", &SparseOptimizerTerminateAction::operator())
      .def("gain_threshold", &SparseOptimizerTerminateAction::gainThreshold)
      .def("set_gain_threshold", &SparseOptimizerTerminateAction::setGainThreshold)
      .def("max_iterations", &SparseOptimizerTerminateAction::maxIterations)
      .def("set_max_iterations", &SparseOptimizerTerminateAction::setMaxIterations);
}

}  // namespace g2o