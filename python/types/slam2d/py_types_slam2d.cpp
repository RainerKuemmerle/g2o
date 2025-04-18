#include "py_types_slam2d.h"

#include "detail/registry.h"
#include "g2o/core/factory.h"
#include "py_parameter_se2_offset.h"
#include "py_vertex_point_xy.h"
#include "py_vertex_se2.h"
#include "types/slam2d/py_edge_pointxy.h"
#include "types/slam2d/py_edge_se2.h"
#include "types/slam2d/py_edge_se2_pointxy.h"
#include "types/slam2d/py_se2.h"

G2O_USE_TYPE_GROUP(slam2d)

namespace g2o {

void declareTypesSlam2d(detail::Registry& registry) {
  declareParameterSE2Offset(registry.mod());

  declareSE2(registry.mod());

  declareVertexPointXY(registry);
  declareVertexSE2(registry);

  declareEdgeSE2(registry);
  declareEdgePointXY(registry);
  declareEdgeSE2PointXY(registry);
}

}  // namespace g2o
