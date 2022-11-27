#pragma once

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2opy.h"

namespace g2o {

inline void declareVertexSE2(py::module& m) {
  py::class_<VertexSE2, BaseVertex<3, SE2>, std::shared_ptr<VertexSE2>>(
      m, "VertexSE2")
      .def(py::init<>())
      .def("set_to_origin_impl", &VertexSE2::setToOriginImpl)
      .def("set_estimate_data_impl", &VertexSE2::setEstimateDataImpl)
      .def("get_estimate_data", &VertexSE2::getEstimateData)
      .def("estimate_dimension", &VertexSE2::estimateDimension)
      .def("set_minimal_estimate_data_impl",
           &VertexSE2::setMinimalEstimateDataImpl)
      .def("get_minimal_estimate_data", &VertexSE2::getMinimalEstimateData)
      .def("minimal_estimate_dimension", &VertexSE2::minimalEstimateDimension);

  // class G2O_TYPES_SLAM2D_API VertexSE2WriteGnuplotAction: public
  // WriteGnuplotAction class G2O_TYPES_SLAM2D_API VertexSE2DrawAction: public
  // DrawAction
}

}  // end namespace g2o
