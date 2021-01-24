#pragma once

#include <pybind11/pybind11.h>

#include "g2o/core/base_edge.h"

namespace g2o {

template <int D, typename E>
void templatedBaseEdge(pybind11::module& m, const std::string& suffix) {
  using namespace pybind11::literals;
  using CLS = BaseEdge<D, E>;

  typedef Eigen::Matrix<number_t, D, 1, Eigen::ColMajor> ErrorVector;
  typedef Eigen::Matrix<number_t, D, D, Eigen::ColMajor> InformationType;

  pybind11::class_<CLS, OptimizableGraph::Edge>(m, ("BaseEdge" + suffix).c_str())
      //.def(pybind11::init<>())
      .def("chi2", &CLS::chi2)
      //.def("error_data", (double* (CLS::*) ()) &CLS::errorData) // -> data*
      .def("error", (ErrorVector & (CLS::*)()) & CLS::error)  // -> ErrorVector

      //.def("information_data", (double* (CLS::*) ()) &CLS::informationData) // -> data*
      .def("information", (InformationType & (CLS::*)()) & CLS::information)  // -> InformationType
      .def("set_information", &CLS::setInformation, "information"_a,
           pybind11::keep_alive<1, 2>())  // InformationType ->

      .def("measurement", &CLS::measurement)                                              // -> E
      .def("set_measurement", &CLS::setMeasurement, "m"_a, pybind11::keep_alive<1, 2>())  // E ->

      .def("rank", &CLS::rank)                         // -> int
      .def("initial_estimate", &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                                       // OptimizableGraph::Vertex*) ->
      ;
}

template <typename E>
void templatedDynamicBaseEdge(pybind11::module& m, const std::string& suffix) {
  using namespace pybind11::literals;
  using CLS = BaseEdge<-1, E>;

  typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor> ErrorVector;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> InformationType;

  pybind11::class_<CLS, OptimizableGraph::Edge>(m, ("DynamicBaseEdge" + suffix).c_str())
      .def("chi2", &CLS::chi2)
      //.def("error_data", (double* (CLS::*) ()) &CLS::errorData) // -> data*
      .def("error", (ErrorVector & (CLS::*)()) & CLS::error)  // -> ErrorVector

      //.def("information_data", (double* (CLS::*) ()) &CLS::informationData) // -> data*
      .def("information", (InformationType & (CLS::*)()) & CLS::information)  // -> InformationType
      .def("set_information", &CLS::setInformation, "information"_a,
           pybind11::keep_alive<1, 2>())  // InformationType ->

      .def("measurement", &CLS::measurement)                                              // -> E
      .def("set_measurement", &CLS::setMeasurement, "m"_a, pybind11::keep_alive<1, 2>())  // E ->

      .def("rank", &CLS::rank)                         // -> int
      .def("initial_estimate", &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                                       // OptimizableGraph::Vertex*) ->
      ;
}

void declareBaseEdge(pybind11::module& m);

}  // end namespace g2o
