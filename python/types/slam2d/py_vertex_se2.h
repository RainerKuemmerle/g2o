#pragma once

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2opy.h"

namespace g2o {

inline void declareVertexSE2(py::module& m) {
  py::class_<VertexSE2, BaseVertex<3, SE2>, std::shared_ptr<VertexSE2>>(
      m, "VertexSE2")
      .def(py::init<>());
  // class G2O_TYPES_SLAM2D_API VertexSE2WriteGnuplotAction: public
  // WriteGnuplotAction class G2O_TYPES_SLAM2D_API VertexSE2DrawAction: public
  // DrawAction
}

}  // end namespace g2o
