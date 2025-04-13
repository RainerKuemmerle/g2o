#pragma once

#include "detail/registry.h"
#include "g2o/types/sclam2d/edge_se2_sensor_calib.h"

namespace g2o {

inline void declareEdgeSE2SensorCalib(detail::Registry& registry) {
  registry.registerEdgeFixed<EdgeSE2SensorCalib>("EdgeSE2SensorCalib");
}

}  // namespace g2o
