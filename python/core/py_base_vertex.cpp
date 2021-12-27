#include "py_base_vertex.h"

namespace g2o {

void declareBaseVertex(pybind11::module& m) {
  // common types
  templatedBaseVertex<1, double>(m, "_1_double");
  templatedBaseVertex<2, Vector2>(m, "_2_Vector2");
  templatedBaseVertex<3, Vector3>(m, "_3_Vector3");
  templatedBaseVertex<4, Eigen::Matrix<double, 5, 1, Eigen::ColMajor>>(
      m, "_4_Vector5");  // sba

  templatedBaseVertex<6, Isometry3>(m, "_6_Isometry3");  // slam3d
}

}  // end namespace g2o
