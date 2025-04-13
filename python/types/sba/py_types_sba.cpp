#include "py_types_sba.h"

#include "detail/registry.h"
#include "g2o/core/factory.h"
#include "g2o/types/sba/edge_project_p2mc.h"
#include "g2o/types/sba/edge_project_p2sc.h"
#include "g2o/types/sba/edge_sba_cam.h"
#include "g2o/types/sba/edge_sba_scale.h"
#include "g2o/types/sba/vertex_intrinsics.h"
#include "g2opy.h"

G2O_USE_TYPE_GROUP(sba)

namespace g2o {

namespace {
void declareVertexIntrinsics(detail::Registry& registry) {
  py::class_<VertexIntrinsicsEstimate>(registry.mod(),
                                       "VertexIntrinsicsEstimate")
      .def(py::init<>())
      .def_readwrite("values", &VertexIntrinsicsEstimate::values);
}
}  // namespace

void declareTypesSBA(detail::Registry& registry) {
  declareVertexIntrinsics(registry);
  registry.registerVertex<VertexIntrinsics>("VertexIntrinsics");
  registry.registerVertex<VertexCam>("VertexCam");

  registry.registerEdgeFixed<EdgeProjectP2MC>("EdgeProjectP2MC");
  registry.registerEdgeFixed<EdgeProjectP2SC>("EdgeProjectP2SC");
  registry.registerEdgeFixed<EdgeSBACam>("EdgeSBACam");
  registry.registerEdgeFixed<EdgeSBAScale>("EdgeSBAScale");
}

}  // end namespace g2o
