#pragma once

#include "detail/registry.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace g2o {

inline void declareVertexSE3(detail::Registry& registry) {
  registry.registerVertex<VertexSE3>("VertexSE3");
}

}  // end namespace g2o
