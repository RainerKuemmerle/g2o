#include "py_types_sclam2d.h"

#include "py_edge_se2_odom_differential_calib.h"
#include "py_edge_se2_sensor_calib.h"
#include "py_odometry_measurement.h"
#include "py_vertex_odom_differential_params.h"

namespace g2o {

void declareTypesSclam2d(py::module& m) {
  declareOdometryMeasurement(m);
  declareVertexOdomDifferentialParams(m);
  declareEdgeSE2SensorCalib(m);
  declareEdgeSE2OdomDifferentialCalib(m);
}

}  // namespace g2o
