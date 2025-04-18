#include "py_types_seven_dof_expmap.h"

#include "detail/registry.h"
#include "g2o/core/factory.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2opy.h"
#include "py_sim3.h"

G2O_USE_TYPE_GROUP(sim3)

namespace g2o {

void declareTypesSevenDofExpmap(detail::Registry& registry) {
  declareSim3(registry.mod());

  registry.registerVertex<VertexSim3Expmap>("VertexSim3Expmap")
      .def("cam_map1", &VertexSim3Expmap::cam_map1)
      .def("cam_map2", &VertexSim3Expmap::cam_map2);

  registry.registerEdgeFixed<EdgeSim3>("EdgeSim3");

  registry.registerEdgeFixed<EdgeSim3ProjectXYZ>("EdgeSim3ProjectXYZ");

  registry.registerEdgeFixed<EdgeInverseSim3ProjectXYZ>(
      "EdgeInverseSim3ProjectXYZ");
}

}  // namespace g2o
