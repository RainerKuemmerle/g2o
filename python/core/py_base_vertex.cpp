#include "py_base_vertex.h"

#include <Eigen/src/Core/util/Constants.h>

namespace g2o {

void declareBaseVertex(pybind11::module& m) {
  // common types
  templatedBaseVertex<1, double>(m, "_1_double");
  templatedBaseVertex<2, Vector2>(m, "_2_Vector2");
  templatedBaseVertex<3, Vector3>(m, "_3_Vector3");
  templatedBaseVertex<4, Vector4>(m, "_4_Vector4");
  templatedBaseVertex<5, Vector5>(m, "_5_Vector5");
  templatedBaseVertex<6, Vector6>(m, "_6_Vector6");

  templatedBaseVertex<Eigen::Dynamic, VectorX>(m, "_Dynamic_VectorX");

  templatedBaseVertex<4, Vector5>(m, "_4_Vector5");  // sba

  templatedBaseVertex<6, Isometry3>(m, "_6_Isometry3");  // slam3d
}

}  // end namespace g2o
