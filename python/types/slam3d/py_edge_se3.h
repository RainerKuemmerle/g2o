#pragma once

#include "detail/registry.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_lotsofxyz.h"
#include "g2o/types/slam3d/edge_se3_offset.h"
#include "g2o/types/slam3d/edge_se3_prior.h"

namespace g2o {

inline void declareEdgeSE3(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgeSE3>("EdgeSE3");
  registry.registerEdgeFixed<EdgeSE3Offset>("EdgeSE3Offset");
  registry.registerEdgeFixed<EdgeSE3Prior>("EdgeSE3Prior");
  registry.registerVariableEdge<EdgeSE3LotsOfXYZ>("EdgeSE3LotsOfXYZ");
}

}  // namespace g2o
