#include "py_base_edge.h"

namespace g2o {

void declareBaseEdge(py::module& m) {
  // common types
  templatedBaseEdge<1, double>(m, "_1_double");
  templatedBaseEdge<2, Vector2>(m, "_2_Vector2");
  templatedBaseEdge<3, Vector3>(m, "_3_Vector3");
  templatedBaseEdge<4, Vector4>(m, "_4_Vector4");

  templatedBaseEdge<6, Isometry3>(m, "_6_Isometry3");

  templatedDynamicBaseEdge<VectorX>(m, "_VectorX");
}

}  // end namespace g2o
