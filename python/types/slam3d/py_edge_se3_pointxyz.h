#pragma once

#include "detail/registry.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_depth.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_disparity.h"

namespace g2o {

inline void declareEdgeSE3PointXYZ(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgeSE3PointXYZ>("EdgeSE3PointXYZ");
  registry.registerEdgeFixed<EdgeSE3PointXYZDepth>("EdgeSE3PointXYZDepth");
  registry.registerEdgeFixed<EdgeSE3PointXYZDisparity>(
      "EdgeSE3PointXYZDisparity");
}

}  // namespace g2o
