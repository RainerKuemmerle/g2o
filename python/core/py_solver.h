#pragma once

#include <g2o/core/solver.h>

#include "g2opy.h"

namespace g2o {

inline void declareSolver(py::module& m) {
  // abstract class
  py::class_<Solver>(m, "Solver")
      .def("x", static_cast<number_t* (Solver::*)()>(&Solver::x))
      .def("b", static_cast<number_t* (Solver::*)()>(&Solver::b))
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
