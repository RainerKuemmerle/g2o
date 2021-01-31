#include "py_types_slam3d.h"

#include "py_edge_pointxyz.h"
#include "py_edge_se3.h"
#include "py_edge_se3_pointxyz.h"
#include "py_parameter.h"
#include "py_se3quat.h"
#include "py_vertex_pointxyz.h"
#include "py_vertex_se3.h"

namespace g2o {

void declareTypesSlam3d(py::module& m) {
  declareSalm3dParameter(m);

  declareSE3Quat(m);
  declareVertexSE3(m);
  declareVertexPointXYZ(m);

  declareEdgePointXYZ(m);
  declareEdgeSE3(m);
  declareEdgeSE3PointXYZ(m);
}

}  // namespace g2o
