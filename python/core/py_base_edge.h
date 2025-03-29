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
      .def_property(
          "error", [](const CLS& cls) { return cls.error(); },
          [](CLS& cls, ErrorVector& v) { cls.error() = v; })
      //.def("information_data", (double* (CLS::*) ()) &CLS::informationData) //
      //-> data*
      .def("information", static_cast<InformationType& (CLS::*)()>(
                              &CLS::information))  // -> InformationType
      .def(
          "set_information",
          [](CLS& edge, const MatrixX& info) { edge.setInformation(info); },
          "information"_a)  // InformationType ->

      .def("measurement", &CLS::measurement)                // -> E
      .def("set_measurement", &CLS::setMeasurement, "m"_a)  // E ->

      .def("rank", &CLS::rank)  // -> int
      .def("initial_estimate",
           &CLS::initialEstimate)  // (const OptimizableGraph::VertexSet&,
                                   // OptimizableGraph::Vertex*) ->
      ;
}

void declareBaseEdge(py::module& m);

}  // end namespace g2o
