#pragma once

#include "detail/registry.h"
#include "g2o/types/sclam2d/edge_se2_odom_differential_calib.h"

namespace g2o {

inline void declareEdgeSE2OdomDifferentialCalib(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgeSE2OdomDifferentialCalib>(
      "EdgeSE2OdomDifferentialCalib");
}

}  // namespace g2o
