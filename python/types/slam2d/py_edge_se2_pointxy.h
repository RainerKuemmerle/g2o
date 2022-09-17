#pragma once

#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2_pointxy_bearing.h"
#include "g2o/types/slam2d/edge_se2_pointxy_calib.h"
#include "g2o/types/slam2d/edge_se2_pointxy_offset.h"
#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"

namespace g2o {

inline void declareEdgeSE2PointXY(py::module& m) {
  templatedBaseBinaryEdge<2, Vector2, VertexSE2, VertexPointXY>(
      m, "_2_Vector2_VertexSE2_VertexPointXY");
  py::class_<EdgeSE2PointXY,
             BaseBinaryEdge<2, Vector2, VertexSE2, VertexPointXY>,
             std::shared_ptr<EdgeSE2PointXY>>(m, "EdgeSE2PointXY")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2PointXY::computeError)
      .def("set_measurement_data", &EdgeSE2PointXY::setMeasurementData)
      .def("get_measurement_data", &EdgeSE2PointXY::getMeasurementData)
      .def("measurement_dimension", &EdgeSE2PointXY::measurementDimension)
      .def("set_measurement_from_state",
           &EdgeSE2PointXY::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE2PointXY::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2PointXY::initialEstimate)
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      .def("linearize_oplus", &EdgeSE2PointXY::linearizeOplus)
#endif
      ;

  // class G2O_TYPES_SLAM2D_API EdgeSE2PointXYWriteGnuplotAction: public
  // WriteGnuplotAction class G2O_TYPES_SLAM2D_API EdgeSE2PointXYDrawAction:
  // public DrawAction

  templatedBaseBinaryEdge<1, double, VertexSE2, VertexPointXY>(
      m, "_1_double_VertexSE2_VertexPointXY");
  py::class_<EdgeSE2PointXYBearing,
             BaseBinaryEdge<1, double, VertexSE2, VertexPointXY>,
             std::shared_ptr<EdgeSE2PointXYBearing>>(m, "EdgeSE2PointXYBearing")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2PointXYBearing::computeError)
      .def("set_measurement_data", &EdgeSE2PointXYBearing::setMeasurementData)
      .def("get_measurement_data", &EdgeSE2PointXYBearing::getMeasurementData)
      .def("measurement_dimension",
           &EdgeSE2PointXYBearing::measurementDimension)
      .def("set_measurement_from_state",
           &EdgeSE2PointXYBearing::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE2PointXYBearing::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2PointXYBearing::initialEstimate);

  // class G2O_TYPES_SLAM2D_API EdgeSE2PointXYBearingWriteGnuplotAction: public
  // WriteGnuplotAction class G2O_TYPES_SLAM2D_API
  // EdgeSE2PointXYBearingDrawAction: public DrawAction

  py::class_<EdgeSE2PointXYCalib, BaseVariableSizedEdge<2, Vector2>,
             std::shared_ptr<EdgeSE2PointXYCalib>>(m, "EdgeSE2PointXYCalib")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2PointXYCalib::computeError)
      .def("initial_estimate_possible",
           &EdgeSE2PointXYCalib::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2PointXYCalib::initialEstimate);

  py::class_<EdgeSE2PointXYOffset,
             BaseBinaryEdge<2, Vector2, VertexSE2, VertexPointXY>,
             std::shared_ptr<EdgeSE2PointXYOffset>>(m, "EdgeSE2PointXYOffset")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2PointXYOffset::computeError)
      .def("set_measurement", &EdgeSE2PointXYOffset::setMeasurement)
      .def("set_measurement_data", &EdgeSE2PointXYOffset::setMeasurementData)
      .def("get_measurement_data", &EdgeSE2PointXYOffset::getMeasurementData)
      .def("measurement_dimension", &EdgeSE2PointXYOffset::measurementDimension)
      .def("set_measurement_from_state",
           &EdgeSE2PointXYOffset::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE2PointXYOffset::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2PointXYOffset::initialEstimate)
      .def("linearize_oplus", &EdgeSE2PointXYOffset::linearizeOplus);
}

}  // namespace g2o
