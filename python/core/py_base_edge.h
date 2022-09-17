#pragma once

#include "g2o/core/base_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2opy.h"

namespace g2o {

template <int D, typename E>
void templatedBaseEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseEdge<D, E>;

  using ErrorVector = typename CLS::ErrorVector;
  using InformationType = typename CLS::InformationType;

  py::class_<CLS, OptimizableGraph::Edge, std::shared_ptr<CLS>>(
      m, ("BaseEdge" + suffix).c_str())
      //.def(py::init<>())
      .def("chi2", &CLS::chi2)
      //.def("error_data", (double* (CLS::*) ()) &CLS::errorData) // -> data*
      .def("error",
           static_cast<ErrorVector& (CLS::*)()>(&CLS::error))  // -> ErrorVector

      //.def("information_data", (double* (CLS::*) ()) &CLS::informationData) //
      //-> data*
      .def("information", static_cast<InformationType& (CLS::*)()>(
                              &CLS::information))  // -> InformationType
      .def("set_information", &CLS::setInformation, "information"_a,
           py::keep_alive<1, 2>())  // InformationType ->

      .def("measurement", &CLS::measurement)  // -> E
      .def("set_measurement", &CLS::setMeasurement, "m"_a,
           py::keep_alive<1, 2>())  // E ->

      .def("rank", &CLS::rank)  // -> int
      .def("initial_estimate",
           &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                   // OptimizableGraph::Vertex*) ->
      ;
}

template <typename E>
void templatedDynamicBaseEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseEdge<-1, E>;

  using ErrorVector = typename CLS::ErrorVector;
  using InformationType = typename CLS::InformationType;

  py::class_<CLS, OptimizableGraph::Edge, std::shared_ptr<CLS>>(
      m, ("DynamicBaseEdge" + suffix).c_str())
      .def("chi2", &CLS::chi2)
      //.def("error_data", (double* (CLS::*) ()) &CLS::errorData) // -> data*
      .def("error",
           static_cast<ErrorVector& (CLS::*)()>(&CLS::error))  // -> ErrorVector

      //.def("information_data", (double* (CLS::*) ()) &CLS::informationData) //
      //-> data*
      .def("information", static_cast<InformationType& (CLS::*)()>(
                              &CLS::information))  // -> InformationType
      .def("set_information", &CLS::setInformation, "information"_a,
           py::keep_alive<1, 2>())  // InformationType ->

      .def("measurement", &CLS::measurement)  // -> E
      .def("set_measurement", &CLS::setMeasurement, "m"_a,
           py::keep_alive<1, 2>())  // E ->

      .def("rank", &CLS::rank)  // -> int
      .def("initial_estimate",
           &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                   // OptimizableGraph::Vertex*) ->
      ;
}

void declareBaseEdge(py::module& m);

}  // end namespace g2o
