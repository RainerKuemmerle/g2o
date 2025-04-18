#pragma once

#include "detail/registry.h"
#include "g2o/types/slam2d/vertex_se2.h"

namespace g2o {

inline void declareVertexSE2(detail::Registry& registry) {
  registry.registerVertex<VertexSE2>("VertexSE2");
}

}  // end namespace g2o
