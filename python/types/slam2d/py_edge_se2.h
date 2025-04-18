#pragma once

#include "detail/registry.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_lotsofxy.h"
#include "g2o/types/slam2d/edge_se2_offset.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/edge_se2_twopointsxy.h"
#include "g2o/types/slam2d/edge_se2_xyprior.h"

namespace g2o {

inline void declareEdgeSE2(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgeSE2>("EdgeSE2");
  registry.registerEdgeFixed<EdgeSE2Offset>("EdgeSE2Offset");
  registry.registerEdgeFixed<EdgeSE2Prior>("EdgeSE2Prior");
  registry.registerEdgeFixed<EdgeSE2TwoPointsXY>("EdgeSE2TwoPointsXY");
  registry.registerEdgeFixed<EdgeSE2XYPrior>("EdgeSE2XYPrior");

  registry.registerVariableEdge<EdgeSE2LotsOfXY>("EdgeSE2LotsOfXY");
}

}  // end namespace g2o
