#pragma once

#include <g2o/core/base_binary_edge.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

template <int D, typename E, typename... VertexTypes>
void templatedBaseFixedSizedEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseFixedSizedEdge<D, E, VertexTypes...>;

  py::class_<CLS, BaseEdge<D, E>>(m, ("BaseFixedSizedEdge" + suffix).c_str())
      //.def(py::init<>())    // lead to "error: invalid new-expression of abstract class type ..."
      .def("create_vertex", &CLS::createVertex, "i"_a)  // -> OptimizableGraph::Vertex*

      .def("resize", &CLS::resize)
      .def("all_vertices_fixed", &CLS::allVerticesFixed)
      .def("linearize_oplus", (void (CLS::*)(JacobianWorkspace&)) & CLS::linearizeOplus)
      .def("linearize_oplus", (void (CLS::*)()) & CLS::linearizeOplus)
      //   .def("jacobian_oplus_xi", &CLS::template jacobianOplusXn<0>())  // ->
      //   JacobianXiOplusType& .def("jacobian_oplus_xj", &CLS::template jacobianOplusXn<1>())  //
      //   -> JacobianXjOplusType&
      .def("construct_quadratic_form", &CLS::constructQuadraticForm)
      .def("map_hessian_memory", &CLS::mapHessianMemory, "d"_a, "i"_a, "j"_a, "row_mayor"_a);
}

void declareBaseFixedSizedEdge(py::module& m) {
  // common types
}

}  // end namespace g2o
