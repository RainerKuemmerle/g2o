#include "py_types_sclam2d.h"

#include "g2o/core/factory.h"
#include "py_edge_se2_odom_differential_calib.h"
#include "py_edge_se2_sensor_calib.h"
#include "py_odometry_measurement.h"
#include "py_vertex_odom_differential_params.h"

G2O_USE_TYPE_GROUP(sclam)

namespace g2o {

void declareTypesSclam2d(py::module& m) {
  declareOdometryMeasurement(m);
  declareVertexOdomDifferentialParams(m);
  declareEdgeSE2SensorCalib(m);
  declareEdgeSE2OdomDifferentialCalib(m);
}

}  // namespace g2o
