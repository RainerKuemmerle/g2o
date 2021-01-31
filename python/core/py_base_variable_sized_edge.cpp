#include "py_base_variable_sized_edge.h"

namespace g2o {

void declareBaseVariableSizedEdge(py::module& m) {
  // common types
  templatedBaseVariableSizedEdge<2, Vector2>(m, "_2_Vector2");
  templatedBaseVariableSizedEdge<3, Vector3>(m, "_3_Vector3");
  templatedBaseVariableSizedEdge<4, Vector4>(m, "_4_Vector4");

  // templatedDynamicBaseVariableSizedEdge<Vector2>(m, "_Vector2");
  // templatedDynamicBaseVariableSizedEdge<Vector3>(m, "_Vector3");
  templatedDynamicBaseVariableSizedEdge<VectorX>(m, "_VectorX");
}

}  // end namespace g2o
