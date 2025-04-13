#pragma once

#include "detail/registry.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2_pointxy_bearing.h"
#include "g2o/types/slam2d/edge_se2_pointxy_calib.h"
#include "g2o/types/slam2d/edge_se2_pointxy_offset.h"

namespace g2o {

inline void declareEdgeSE2PointXY(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgeSE2PointXY>("EdgeSE2PointXY");
  registry.registerEdgeFixed<EdgeSE2PointXYBearing>("EdgeSE2PointXYBearing");
  registry.registerEdgeFixed<EdgeSE2PointXYOffset>("EdgeSE2PointXYOffset");

  registry.registerEdgeFixed<EdgeSE2PointXYCalib>("EdgeSE2PointXYCalib");
}

}  // namespace g2o
