#pragma once

#include "detail/registry.h"
#include "g2o/types/slam3d/edge_pointxyz.h"

namespace g2o {

inline void declareEdgePointXYZ(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgePointXYZ>("EdgePointXYZ");
}

}  // namespace g2o
