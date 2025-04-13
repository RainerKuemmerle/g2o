#pragma once

#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2_pointxy_bearing.h"
#include "g2o/types/slam2d/edge_se2_pointxy_calib.h"
#include "g2o/types/slam2d/edge_se2_pointxy_offset.h"
#include "python/detail/register_edge.h"

namespace g2o {

inline void declareEdgeSE2PointXY(py::module& m) {
  registerEdgeBinary<EdgeSE2PointXY>(m, "EdgeSE2PointXY");
  registerEdgeBinary<EdgeSE2PointXYBearing>(m, "EdgeSE2PointXYBearing");
  registerEdgeBinary<EdgeSE2PointXYOffset>(m, "EdgeSE2PointXYOffset");

  templatedBaseFixedSizedEdge<2, Vector2, VertexSE2, VertexPointXY, VertexSE2>(
      m, "_2_Vector2_VertexSE2_VertexPointXY_VertexSE2");
  py::class_<
      EdgeSE2PointXYCalib,
      BaseFixedSizedEdge<2, Vector2, VertexSE2, VertexPointXY, VertexSE2>,
      std::shared_ptr<EdgeSE2PointXYCalib>>(m, "EdgeSE2PointXYCalib")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2PointXYCalib::computeError)
      .def("initial_estimate_possible",
           &EdgeSE2PointXYCalib::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2PointXYCalib::initialEstimate);
}

}  // namespace g2o
