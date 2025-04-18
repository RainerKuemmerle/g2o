#pragma once

#include <g2o/types/sclam2d/vertex_odom_differential_params.h>

#include "detail/registry.h"

namespace g2o {

inline void declareVertexOdomDifferentialParams(detail::Registry& registry) {
  registry.registerVertex<VertexOdomDifferentialParams>(
      "VertexOdomDifferentialParams");
}

}  // namespace g2o
