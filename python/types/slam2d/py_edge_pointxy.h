#pragma once

#include "detail/registry.h"
#include "g2o/types/slam2d/edge_pointxy.h"

namespace g2o {

inline void declareEdgePointXY(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgePointXY>("EdgePointXY");
}

}  // end namespace g2o
