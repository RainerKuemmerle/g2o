#pragma once

#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2opy.h"

namespace g2o {

inline void declareVertexPointXYZ(py::module& m) {
  py::class_<VertexPointXYZ, BaseVertex<3, Vector3>,
             std::shared_ptr<VertexPointXYZ>>(m, "VertexPointXYZ")
      .def(py::init<>());
  // class G2O_TYPES_SLAM3D_API VertexPointXYZWriteGnuplotAction: public
  // WriteGnuplotAction class VertexPointXYZDrawAction: public DrawAction
}

}  // end namespace g2o
