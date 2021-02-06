#include "py_types_slam2d.h"

#include "py_edge_pointxy.h"
#include "py_edge_se2.h"
#include "py_edge_se2_pointxy.h"
#include "py_parameter_se2_offset.h"
#include "py_se2.h"
#include "py_vertex_point_xy.h"
#include "py_vertex_se2.h"

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
