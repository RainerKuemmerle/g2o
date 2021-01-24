#include <pybind11/pybind11.h>

#include "g2o/types/sclam2d/edge_se2_odom_differential_calib.h"
#include "python/core/py_base_fixed_sized_edge.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareEdgeSE2OdomDifferentialCalib(py::module& m) {
  templatedBaseFixedSizedEdge<3, VelocityMeasurement, VertexSE2, VertexSE2,
                              VertexOdomDifferentialParams>(
      m, "_3_VelocityMeasurement_VertexSE2_VertexSE2_VertexOdomDifferentialParams");

  py::class_<EdgeSE2OdomDifferentialCalib,
             BaseFixedSizedEdge<3, VelocityMeasurement, VertexSE2, VertexSE2,
                                VertexOdomDifferentialParams>>(m, "EdgeSE2OdomDifferentialCalib")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2OdomDifferentialCalib::computeError);

  // class G2O_TYPES_SCLAM2D_API EdgeSE2OdomDifferentialCalibDrawAction: public DrawAction
}

}  // namespace g2o
