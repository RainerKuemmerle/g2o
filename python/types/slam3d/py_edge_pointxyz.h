#pragma once

#include "g2o/types/slam3d/edge_pointxyz.h"
#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"

namespace g2o {

inline void declareEdgePointXYZ(py::module& m) {
  templatedBaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexPointXYZ>(
      m, "_3_Vector3_VertexPointXYZ_VertexPointXYZ");

  py::class_<EdgePointXYZ,
             BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexPointXYZ>,
             std::shared_ptr<EdgePointXYZ>>(m, "EdgePointXYZ")
      .def(py::init<>())

      .def("compute_error", &EdgePointXYZ::computeError)
      .def("set_measurement", &EdgePointXYZ::setMeasurement)
      .def("set_measurement_data", &EdgePointXYZ::setMeasurementData)
      .def("get_measurement_data", &EdgePointXYZ::getMeasurementData)
      .def("measurement_dimension", &EdgePointXYZ::measurementDimension)
      .def("set_measurement_from_state", &EdgePointXYZ::setMeasurementFromState)
      .def("initial_estimate_possible", &EdgePointXYZ::initialEstimatePossible)
#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
      .def("linearize_oplus", &EdgePointXYZ::linearizeOplus)
#endif
      ;
}

}  // namespace g2o
