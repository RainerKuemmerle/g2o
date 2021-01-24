#include <g2o/core/solver.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareSolver(py::module& m) {
  // abstract class
  py::class_<Solver>(m, "Solver")
      .def("x", (double* (Solver::*)()) & Solver::x)
      .def("b", (double* (Solver::*)()) & Solver::b)
      .def("vector_size", &Solver::vectorSize)
      .def("optimizer", &Solver::optimizer)
      .def("set_optimizer", &Solver::setOptimizer)
      .def("levenberg", &Solver::levenberg)
      .def("set_levenberg", &Solver::setLevenberg)
      .def("supports_schur", &Solver::supportsSchur)
      .def("additional_vector_space", &Solver::additionalVectorSpace)
      .def("set_additional_vector_space", &Solver::setAdditionalVectorSpace);
}

}  // end namespace g2o