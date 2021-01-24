#pragma once

#include <g2o/core/base_multi_edge.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

template <int D, typename E>
void templatedBaseMultiEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseMultiEdge<D, E>;

  // typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
  // typedef typename BaseEdge<D,E>::InformationType InformationType;

  py::class_<CLS, BaseEdge<D, E>>(m, ("BaseMultiEdge" + suffix).c_str())
      .def("resize", &CLS::resize, "size"_a)              // size_t ->
      .def("all_vertices_fixed", &CLS::allVerticesFixed)  // -> bool
      .def("construct_quadratic_form", &CLS::constructQuadraticForm)
      .def("linearize_oplus", (void (CLS::*)(JacobianWorkspace&)) & CLS::linearizeOplus)
      .def("linearize_oplus", (void (CLS::*)()) & CLS::linearizeOplus)
      .def("mapHessianMemory", &CLS::mapHessianMemory, "d"_a, "i"_a, "j"_a,
           "row_major"_a)  // (double*, i, j, bool) ->

      ;
}

template <typename E>
void templatedDynamicBaseMultiEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseMultiEdge<-1, E>;

  // typedef typename BaseEdge<-1,E>::ErrorVector ErrorVector;
  // typedef typename BaseEdge<-1,E>::InformationType InformationType;

  py::class_<CLS, BaseEdge<-1, E>>(m, ("DynamicBaseMultiEdge" + suffix).c_str())
      .def("resize", &CLS::resize, "size"_a)              // size_t ->
      .def("all_vertices_fixed", &CLS::allVerticesFixed)  // -> bool
      .def("construct_quadratic_form", &CLS::constructQuadraticForm)
      .def("linearize_oplus", (void (CLS::*)(JacobianWorkspace&)) & CLS::linearizeOplus)
      .def("linearize_oplus", (void (CLS::*)()) & CLS::linearizeOplus)
      .def("mapHessianMemory", &CLS::mapHessianMemory, "d"_a, "i"_a, "j"_a,
           "row_major"_a)  // (double*, i, j, bool) ->

      ;
}

void declareBaseMultiEdge(py::module& m);

}  // end namespace g2o
