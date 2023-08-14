#pragma once

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2opy.h"

namespace g2o {

inline void declareVertexSE3(py::module& m) {
  py::class_<VertexSE3, BaseVertex<6, Isometry3>, std::shared_ptr<VertexSE3>>(
      m, "VertexSE3")
      .def(py::init<>());
}

}  // end namespace g2o
