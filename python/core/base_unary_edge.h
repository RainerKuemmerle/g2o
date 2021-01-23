#pragma once

#include <g2o/core/base_unary_edge.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

template <int D, typename E, typename VertexXiType>
void templatedBaseUnaryEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseUnaryEdge<D, E, VertexXiType>;

  py::class_<CLS, BaseEdge<D, E>>(m, ("BaseUnaryEdge" + suffix).c_str())
      .def("resize", &CLS::resize, "size"_a)              // size_t ->
      .def("all_vertices_fixed", &CLS::allVerticesFixed)  // -> bool
      .def("create_vertex", &CLS::createVertex, "i"_a)    // int -> OptimizableGraph::Vertex*
      .def("construct_quadratic_form", &CLS::constructQuadraticForm)
      .def("linearize_oplus", (void (CLS::*)(JacobianWorkspace&)) & CLS::linearizeOplus)
      .def("linearize_oplus", (void (CLS::*)()) & CLS::linearizeOplus)
      .def("initial_estimate", &CLS::initialEstimate);  // (const OptimizableGraph::VertexSet&,
                                                        // OptimizableGraph::Vertex*) ->
}

void declareBaseUnaryEdge(py::module& m) {
  // common types
}

}  // end namespace g2o