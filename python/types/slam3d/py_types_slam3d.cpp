#include "py_types_slam3d.h"

#include "g2o/core/factory.h"
#include "py_edge_pointxyz.h"
#include "py_edge_se3.h"
#include "py_edge_se3_pointxyz.h"
#include "py_parameter.h"
#include "py_se3quat.h"
#include "py_vertex_pointxyz.h"
#include "py_vertex_se3.h"

G2O_USE_TYPE_GROUP(slam3d)

namespace g2o {

void declareTypesSlam3d(detail::Registry& registry) {
  declareSalm3dParameter(registry);

  declareSE3Quat(registry);

  declareVertexSE3(registry);
  declareVertexPointXYZ(registry);

  declareEdgePointXYZ(registry);
  declareEdgeSE3(registry);
  declareEdgeSE3PointXYZ(registry);
}

}  // namespace g2o
