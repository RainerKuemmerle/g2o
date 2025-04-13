#pragma once

#include "detail/registry.h"
#include "g2o/types/slam2d/vertex_point_xy.h"

namespace g2o {

inline void declareVertexPointXY(detail::Registry& registry) {
  registry.registerVertex<VertexPointXY>("VertexPointXY");
}

}  // end namespace g2o
