#pragma once

#include "g2o/types/slam3d/vertex_se3.h"
#include "g2opy.h"

namespace g2o {

inline void declareVertexSE3(py::module& m) {
  py::class_<VertexSE3, BaseVertex<6, Isometry3>, std::shared_ptr<VertexSE3>>(
      m, "VertexSE3")
      .def(py::init<>())

      .def("set_to_origin_impl", &VertexSE3::setToOriginImpl)
      .def("set_estimate_data_impl", &VertexSE3::setEstimateDataImpl)
      .def("get_estimate_data", &VertexSE3::getEstimateData)
      .def("estimate_dimension", &VertexSE3::estimateDimension)
      .def("set_minimal_estimate_data_impl",
           &VertexSE3::setMinimalEstimateDataImpl)
      .def("get_minimal_estimate_data", &VertexSE3::getMinimalEstimateData)
      .def("minimal_estimate_dimension", &VertexSE3::minimalEstimateDimension);
}

}  // end namespace g2o
