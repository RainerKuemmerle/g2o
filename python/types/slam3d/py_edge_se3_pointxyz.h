#pragma once

#include "python/core/py_base_binary_edge.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_depth.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_disparity.h"
#include "g2opy.h"

namespace g2o {

inline void declareEdgeSE3PointXYZ(py::module& m) {
  templatedBaseBinaryEdge<3, Vector3, VertexSE3, VertexPointXYZ>(
      m, "_3_Vector3_VertexSE3_VertexPointXYZ");
  py::class_<EdgeSE3PointXYZ,
             BaseBinaryEdge<3, Vector3, VertexSE3, VertexPointXYZ>,
             std::shared_ptr<EdgeSE3PointXYZ>>(m, "EdgeSE3PointXYZ")
      .def(py::init<>())

      .def("compute_error", &EdgeSE3PointXYZ::computeError)
      .def("linearize_oplus", &EdgeSE3PointXYZ::linearizeOplus)
      .def("set_measurement", &EdgeSE3PointXYZ::setMeasurement)
      .def("set_measurement_data", &EdgeSE3PointXYZ::setMeasurementData)
      .def("get_measurement_data", &EdgeSE3PointXYZ::getMeasurementData)
      .def("measurement_dimension", &EdgeSE3PointXYZ::measurementDimension)
      .def("set_measurement_from_state",
           &EdgeSE3PointXYZ::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE3PointXYZ::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE3PointXYZ::initialEstimate);

  // class EdgeSE3PointXYZDrawAction: public DrawAction

  py::class_<EdgeSE3PointXYZDepth,
             BaseBinaryEdge<3, Vector3, VertexSE3, VertexPointXYZ>,
             std::shared_ptr<EdgeSE3PointXYZDepth>>(m, "EdgeSE3PointXYZDepth")
      .def(py::init<>())

      .def("compute_error", &EdgeSE3PointXYZDepth::computeError)
      .def("linearize_oplus", &EdgeSE3PointXYZDepth::linearizeOplus)
      .def("set_measurement", &EdgeSE3PointXYZDepth::setMeasurement)
      .def("set_measurement_data", &EdgeSE3PointXYZDepth::setMeasurementData)
      .def("get_measurement_data", &EdgeSE3PointXYZDepth::getMeasurementData)
      .def("measurement_dimension", &EdgeSE3PointXYZDepth::measurementDimension)
      .def("set_measurement_from_state",
           &EdgeSE3PointXYZDepth::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE3PointXYZDepth::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE3PointXYZDepth::initialEstimate);

  py::class_<EdgeSE3PointXYZDisparity,
             BaseBinaryEdge<3, Vector3, VertexSE3, VertexPointXYZ>,
             std::shared_ptr<EdgeSE3PointXYZDisparity>>(
      m, "EdgeSE3PointXYZDisparity")
      .def(py::init<>())

      .def("compute_error", &EdgeSE3PointXYZDisparity::computeError)
#ifdef EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN
      .def("linearize_oplus", &EdgeSE3PointXYZDisparity::linearizeOplus)
#endif
      .def("set_measurement", &EdgeSE3PointXYZDisparity::setMeasurement)
      .def("set_measurement_data",
           &EdgeSE3PointXYZDisparity::setMeasurementData)
      .def("get_measurement_data",
           &EdgeSE3PointXYZDisparity::getMeasurementData)
      .def("measurement_dimension",
           &EdgeSE3PointXYZDisparity::measurementDimension)
      .def("set_measurement_from_state",
           &EdgeSE3PointXYZDisparity::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE3PointXYZDisparity::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE3PointXYZDisparity::initialEstimate);
}

}  // namespace g2o
