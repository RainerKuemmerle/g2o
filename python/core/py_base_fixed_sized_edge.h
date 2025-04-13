#pragma once

#include <pybind11/detail/common.h>

#include "g2o/core/base_fixed_sized_edge.h"
#include "g2opy.h"

namespace g2o {

template <int D, typename E, typename... VertexTypes>
void templatedBaseFixedSizedEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseFixedSizedEdge<D, E, VertexTypes...>;

  py::class_<CLS, BaseEdge<D, E>, std::shared_ptr<CLS>>(
      m, ("BaseFixedSizedEdge" + suffix).c_str())
      // abstract class type ..."
      // TODO(Rainer): Fix binding of create_vertex
      //  .def("create_vertex", &CLS::createVertex,
      //       "i"_a)  // -> OptimizableGraph::Vertex*
      .def("resize", &CLS::resize)
      .def("all_vertices_fixed", &CLS::allVerticesFixed)
      .def("compute_error", &CLS::computeError)
      .def("linearize_oplus",
           static_cast<void (CLS::*)(JacobianWorkspace&)>(&CLS::linearizeOplus))
      .def("linearize_oplus",
           static_cast<void (CLS::*)()>(&CLS::linearizeOplus))
      .def("set_measurement_data", &CLS::setMeasurementData)
      .def("get_measurement_data", &CLS::getMeasurementData)
      .def("measurement_dimension", &CLS::measurementDimension)
      .def("set_measurement_from_state", &CLS::setMeasurementFromState)
      .def("initial_estimate_possible", &CLS::initialEstimatePossible)
      .def("initial_estimate", &CLS::initialEstimate)
      .def("construct_quadratic_form", &CLS::constructQuadraticForm)
      .def("map_hessian_memory", &CLS::mapHessianMemory, "d"_a, "i"_a, "j"_a,
           "row_mayor"_a)
      .def("jacobian", &CLS::jacobian, "vertex_index"_a)  // int -> Matrix
      .def("set_jacobian", &CLS::setJacobian, "vertex_index"_a,
           "jacobian"_a)  // int, Matrix ->
      ;
}

}  // end namespace g2o
