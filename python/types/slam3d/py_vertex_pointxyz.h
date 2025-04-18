#pragma once

#include "detail/registry.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

namespace g2o {

inline void declareVertexPointXYZ(detail::Registry& registry) {
  registry.registerVertex<VertexPointXYZ>("VertexPointXYZ");
}

}  // end namespace g2o
