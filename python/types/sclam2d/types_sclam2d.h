#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/sclam2d/types_sclam2d.h>
#include <pybind11/pybind11.h>

#include "edge_se2_odom_differential_calib.h"
#include "edge_se2_sensor_calib.h"
#include "odometry_measurement.h"
#include "vertex_odom_differential_params.h"

namespace g2o {

void declareTypesSclam2d(py::module& m) {
  declareOdometryMeasurement(m);
  declareVertexOdomDifferentialParams(m);
  declareEdgeSE2SensorCalib(m);
  declareEdgeSE2OdomDifferentialCalib(m);
}

}  // namespace g2o
