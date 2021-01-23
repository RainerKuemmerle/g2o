#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <pybind11/pybind11.h>

#include "edge_pointxyz.h"
#include "edge_se3.h"
#include "edge_se3_pointxyz.h"
#include "parameter.h"
#include "se3quat.h"
#include "vertex_pointxyz.h"
#include "vertex_se3.h"

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
