#pragma once

#include "g2o/core/base_vertex.h"
#include "g2o/core/eigen_types.h"
#include "g2opy.h"

namespace g2o {

template <int D, typename T>
void templatedBaseVertex(py::module& m, const std::string& suffix) {
  using CLS = BaseVertex<D, T>;
  using BVector = typename CLS::BVector;

  py::class_<CLS, OptimizableGraph::Vertex, std::shared_ptr<CLS>>(
      m, ("BaseVertex" + suffix).c_str())

      //.def(py::init<>())
      //.def_readonly_static("dimension", &BaseVertex<D, T>::Dimension)   //
      // lead to undefined
      // symbol error
      .def("hessian", [](const CLS& v) { return MatrixX(v.hessianMap()); })
      //-> double* .def("map_hessian_memory", &CLS::mapHessianMemory) // double*
      //-> void .def("copy_b", &CLS::copyB) // double* -> void
      .def("clear_quadratic_form", &CLS::clearQuadraticForm)
      .def("solve_direct", &CLS::solveDirect)
      .def("b", static_cast<BVector& (CLS::*)()>(&CLS::b),
           py::return_value_policy::reference)
      //.def("A", (HessianBlockType& (CLS::*) ()) &CLS::A,
      //        py::return_value_policy::reference)

      .def("push", &CLS::push)
      .def("pop", &CLS::pop)
      .def("discard_top", &CLS::discardTop)
      .def("stack_size", &CLS::stackSize)  // -> int

      .def("estimate", &CLS::estimate,
           py::return_value_policy::reference)         // -> T&
      .def("set_estimate", &CLS::setEstimate, "et"_a)  // T& -> void
      ;
}

void declareBaseVertex(py::module& m);

}  // end namespace g2o
