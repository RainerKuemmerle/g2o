#pragma once

#include "g2o/core/base_binary_edge.h"
#include "g2opy.h"

namespace g2o {

template <int D, typename E, typename... VertexTypes>
void templatedBaseFixedSizedEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseFixedSizedEdge<D, E, VertexTypes...>;

  py::class_<CLS, BaseEdge<D, E>, std::shared_ptr<CLS>>(
      m, ("BaseFixedSizedEdge" + suffix).c_str())
      //.def(py::init<>())    // lead to "error: invalid new-expression of
      // abstract class type ..."
      // TODO(Rainer): Fix binding of create_vertex
      //  .def("create_vertex", &CLS::createVertex,
      //       "i"_a)  // -> OptimizableGraph::Vertex*
      .def("resize", &CLS::resize)
      .def("all_vertices_fixed", &CLS::allVerticesFixed)
      .def("linearize_oplus",
           static_cast<void (CLS::*)(JacobianWorkspace&)>(&CLS::linearizeOplus))
      .def("linearize_oplus",
           static_cast<void (CLS::*)()>(&CLS::linearizeOplus))
      .def("construct_quadratic_form", &CLS::constructQuadraticForm)
      .def("map_hessian_memory", &CLS::mapHessianMemory, "d"_a, "i"_a, "j"_a,
           "row_mayor"_a);
}

}  // end namespace g2o
