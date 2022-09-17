#pragma once

#include "g2o/types/slam2d/edge_pointxy.h"
#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"

namespace g2o {

inline void declareEdgePointXY(py::module& m) {
  templatedBaseBinaryEdge<2, Vector2, VertexPointXY, VertexPointXY>(
      m, "_2_Vector2_VertexPointXY_VertexPointXY");

  py::class_<EdgePointXY,
             BaseBinaryEdge<2, Vector2, VertexPointXY, VertexPointXY>,
             std::shared_ptr<EdgePointXY>>(m, "EdgePointXY")
      .def(py::init<>())
      .def("compute_error", &EdgePointXY::computeError)
      .def("set_measurement", &EdgePointXY::setMeasurement)
      .def("set_measurement_data", &EdgePointXY::setMeasurementData)
      .def("get_measurement_data", &EdgePointXY::getMeasurementData)
      .def("measurement_dimension", &EdgePointXY::measurementDimension)
      .def("set_measurement_from_state", &EdgePointXY::setMeasurementFromState)
      .def("initial_estimate_possible", &EdgePointXY::initialEstimatePossible)
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      .def("linearize_oplus", &EdgePointXY::linearizeOplus)
#endif
      ;
}

}  // end namespace g2o
