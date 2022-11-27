#pragma once

#include "g2o/types/sclam2d/edge_se2_sensor_calib.h"
#include "g2opy.h"
#include "python/core/py_base_fixed_sized_edge.h"

namespace g2o {

inline void declareEdgeSE2SensorCalib(py::module& m) {
  templatedBaseFixedSizedEdge<3, SE2, VertexSE2, VertexSE2, VertexSE2>(
      m, "_3_SE2_VertexSE2_VertexSE2_VertexSE2");

  py::class_<EdgeSE2SensorCalib,
             BaseFixedSizedEdge<3, SE2, VertexSE2, VertexSE2, VertexSE2>,
             std::shared_ptr<EdgeSE2SensorCalib>>(m, "EdgeSE2SensorCalib")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2SensorCalib::computeError)
      .def("set_measurement", &EdgeSE2SensorCalib::setMeasurement)
      .def("initial_estimate_possible",
           &EdgeSE2SensorCalib::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2SensorCalib::initialEstimate);

  // class EdgeSE2SensorCalibDrawAction: public DrawAction
}

}  // namespace g2o
