#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include <pybind11/pybind11.h>

#include "edge_pointxy.h"
#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "parameter_se2_offset.h"
#include "se2.h"
#include "vertex_point_xy.h"
#include "vertex_se2.h"

namespace g2o {

void declareTypesSlam2d(py::module& m) {
  declareParameterSE2Offset(m);

  declareSE2(m);
  declareVertexPointXY(m);
  declareVertexSE2(m);

  declareEdgePointXY(m);
  declareEdgeSE2(m);
  declareEdgeSE2PointXY(m);
}

}  // namespace g2o
