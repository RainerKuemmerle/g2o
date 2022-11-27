#pragma once

#include "g2o/types/sclam2d/edge_se2_odom_differential_calib.h"
#include "g2opy.h"
#include "python/core/py_base_fixed_sized_edge.h"

namespace g2o {

inline void declareEdgeSE2OdomDifferentialCalib(py::module& m) {
  templatedBaseFixedSizedEdge<3, VelocityMeasurement, VertexSE2, VertexSE2,
                              VertexOdomDifferentialParams>(
      m,
      "_3_VelocityMeasurement_VertexSE2_VertexSE2_"
      "VertexOdomDifferentialParams");

  py::class_<EdgeSE2OdomDifferentialCalib,
             BaseFixedSizedEdge<3, VelocityMeasurement, VertexSE2, VertexSE2,
                                VertexOdomDifferentialParams>,
             std::shared_ptr<EdgeSE2OdomDifferentialCalib>>(
      m, "EdgeSE2OdomDifferentialCalib")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2OdomDifferentialCalib::computeError);

  // class G2O_TYPES_SCLAM2D_API EdgeSE2OdomDifferentialCalibDrawAction: public
  // DrawAction
}

}  // namespace g2o
