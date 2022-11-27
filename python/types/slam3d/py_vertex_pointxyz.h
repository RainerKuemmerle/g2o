#pragma once

#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2opy.h"

namespace g2o {

inline void declareVertexPointXYZ(py::module& m) {
  py::class_<VertexPointXYZ, BaseVertex<3, Vector3>,
             std::shared_ptr<VertexPointXYZ>>(m, "VertexPointXYZ")
      .def(py::init<>())

      .def("set_to_origin_impl", &VertexPointXYZ::setToOriginImpl)
      .def("set_estimate_data_impl", &VertexPointXYZ::setEstimateDataImpl)
      .def("get_estimate_data", &VertexPointXYZ::getEstimateData)
      .def("estimate_dimension", &VertexPointXYZ::estimateDimension)
      .def("set_minimal_estimate_data_impl",
           &VertexPointXYZ::setMinimalEstimateDataImpl)
      .def("get_minimal_estimate_data", &VertexPointXYZ::getMinimalEstimateData)
      .def("minimal_estimate_dimension",
           &VertexPointXYZ::minimalEstimateDimension);

  // class G2O_TYPES_SLAM3D_API VertexPointXYZWriteGnuplotAction: public
  // WriteGnuplotAction class VertexPointXYZDrawAction: public DrawAction
}

}  // end namespace g2o
