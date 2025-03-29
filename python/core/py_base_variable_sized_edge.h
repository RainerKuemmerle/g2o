#pragma once

#include "g2o/core/base_variable_sized_edge.h"
#include "g2opy.h"

namespace g2o {

template <int D, typename E>
void templatedBaseVariableSizedEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseVariableSizedEdge<D, E>;

  // typedef typename BaseEdge<D,E>::ErrorVector ErrorVector;
  // typedef typename BaseEdge<D,E>::InformationType InformationType;

  py::class_<CLS, BaseEdge<D, E>, std::shared_ptr<CLS>>(
      m, ("BaseVariableSizedEdge" + suffix).c_str())
      .def("resize", &CLS::resize, "size"_a)              // size_t ->
      .def("all_vertices_fixed", &CLS::allVerticesFixed)  // -> bool
      .def("construct_quadratic_form", &CLS::constructQuadraticForm)
      .def("linearize_oplus",
           static_cast<void (CLS::*)(JacobianWorkspace&)>(&CLS::linearizeOplus))
      .def("linearize_oplus",
           static_cast<void (CLS::*)()>(&CLS::linearizeOplus))
      .def("mapHessianMemory", &CLS::mapHessianMemory, "d"_a, "i"_a, "j"_a,
           "row_major"_a)  // (double*, i, j, bool) ->
      ;
}

template <typename E>
void templatedDynamicBaseVariableSizedEdge(py::module& m,
                                           const std::string& suffix) {
  using CLS = BaseVariableSizedEdge<Eigen::Dynamic, E>;

  py::class_<CLS, BaseEdge<Eigen::Dynamic, E>, std::shared_ptr<CLS>>(
      m, ("DynamicBaseVariableSizedEdge" + suffix).c_str())
      .def("resize", &CLS::resize, "size"_a)              // size_t ->
      .def("all_vertices_fixed", &CLS::allVerticesFixed)  // -> bool
      .def("construct_quadratic_form", &CLS::constructQuadraticForm)
      .def("linearize_oplus",
           static_cast<void (CLS::*)(JacobianWorkspace&)>(&CLS::linearizeOplus))
      .def("linearize_oplus",
           static_cast<void (CLS::*)()>(&CLS::linearizeOplus))
      .def("mapHessianMemory", &CLS::mapHessianMemory, "d"_a, "i"_a, "j"_a,
           "row_major"_a)  // (double*, i, j, bool) ->
      .def("jacobian", &CLS::jacobian, "vertex_index"_a)  // int -> Matrix
      .def("set_jacobian", &CLS::setJacobian, "vertex_index"_a,
           "jacobian"_a)  // int, Matrix ->
      ;
}

void declareBaseVariableSizedEdge(py::module& m);

}  // end namespace g2o
