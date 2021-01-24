#include "py_base_multi_edge.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareBaseMultiEdge(py::module& m) {
  // common types
  templatedBaseMultiEdge<2, Vector2>(m, "_2_Vector2");
  templatedBaseMultiEdge<3, Vector3>(m, "_3_Vector3");
  templatedBaseMultiEdge<4, Vector4>(m, "_4_Vector4");

  // templatedDynamicBaseMultiEdge<Vector2>(m, "_Vector2");
  // templatedDynamicBaseMultiEdge<Vector3>(m, "_Vector3");
  templatedDynamicBaseMultiEdge<VectorX>(m, "_VectorX");
}

}  // end namespace g2o
