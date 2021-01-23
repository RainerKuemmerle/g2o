#pragma once

#include <g2o/core/base_edge.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

template <int D, typename E>
void templatedBaseEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseEdge<D, E>;

  typedef Eigen::Matrix<number_t, D, 1, Eigen::ColMajor> ErrorVector;
  typedef Eigen::Matrix<number_t, D, D, Eigen::ColMajor> InformationType;

  py::class_<CLS, OptimizableGraph::Edge>(m, ("BaseEdge" + suffix).c_str())
      //.def(py::init<>())
      .def("chi2", &CLS::chi2)
      //.def("error_data", (double* (CLS::*) ()) &CLS::errorData) // -> data*
      .def("error", (ErrorVector & (CLS::*)()) & CLS::error)  // -> ErrorVector

      //.def("information_data", (double* (CLS::*) ()) &CLS::informationData) // -> data*
      .def("information", (InformationType & (CLS::*)()) & CLS::information)  // -> InformationType
      .def("set_information", &CLS::setInformation, "information"_a,
           py::keep_alive<1, 2>())  // InformationType ->

      .def("measurement", &CLS::measurement)                                        // -> E
      .def("set_measurement", &CLS::setMeasurement, "m"_a, py::keep_alive<1, 2>())  // E ->

      .def("rank", &CLS::rank)                         // -> int
      .def("initial_estimate", &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                                       // OptimizableGraph::Vertex*) ->
      ;
}

template <typename E>
void templatedDynamicBaseEdge(py::module& m, const std::string& suffix) {
  using CLS = BaseEdge<-1, E>;

  typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor> ErrorVector;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> InformationType;

  py::class_<CLS, OptimizableGraph::Edge>(m, ("DynamicBaseEdge" + suffix).c_str())
      .def("chi2", &CLS::chi2)
      //.def("error_data", (double* (CLS::*) ()) &CLS::errorData) // -> data*
      .def("error", (ErrorVector & (CLS::*)()) & CLS::error)  // -> ErrorVector

      //.def("information_data", (double* (CLS::*) ()) &CLS::informationData) // -> data*
      .def("information", (InformationType & (CLS::*)()) & CLS::information)  // -> InformationType
      .def("set_information", &CLS::setInformation, "information"_a,
           py::keep_alive<1, 2>())  // InformationType ->

      .def("measurement", &CLS::measurement)                                        // -> E
      .def("set_measurement", &CLS::setMeasurement, "m"_a, py::keep_alive<1, 2>())  // E ->

      .def("rank", &CLS::rank)                         // -> int
      .def("initial_estimate", &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                                       // OptimizableGraph::Vertex*) ->
      ;
}

// template<typename E>
// class BaseEdge<-1,E> : public OptimizableGraph::Edge

void declareBaseEdge(py::module& m) {
  // common types
  templatedBaseEdge<1, double>(m, "_1_double");
  templatedBaseEdge<2, Vector2>(m, "_2_Vector2");
  templatedBaseEdge<3, Vector3>(m, "_3_Vector3");
  templatedBaseEdge<4, Vector4>(m, "_4_Vector4");

  templatedBaseEdge<6, Isometry3>(m, "_6_Isometry3");

  templatedDynamicBaseEdge<VectorX>(m, "_VectorX");
}

}  // end namespace g2o
