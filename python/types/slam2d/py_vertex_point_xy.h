#pragma once

#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2opy.h"

namespace g2o {

inline void declareVertexPointXY(py::module& m) {
  py::class_<VertexPointXY, BaseVertex<2, Vector2>,
             std::shared_ptr<VertexPointXY>>(m, "VertexPointXY")
      .def(py::init<>());
  // class G2O_TYPES_SLAM2D_API VertexPointXYWriteGnuplotAction: public
  // WriteGnuplotAction class G2O_TYPES_SLAM2D_API VertexPointXYDrawAction:
  // public DrawAction
}

}  // end namespace g2o
