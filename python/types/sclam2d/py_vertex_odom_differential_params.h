#pragma once

#include <g2o/types/sclam2d/vertex_odom_differential_params.h>

#include "g2opy.h"

namespace g2o {

void declareVertexOdomDifferentialParams(py::module& m) {
  py::class_<VertexOdomDifferentialParams, BaseVertex<3, Vector3>,
             std::shared_ptr<VertexOdomDifferentialParams>>(m, "VertexOdomDifferentialParams")
      .def(py::init<>())
      .def("set_to_origin_impl", &VertexOdomDifferentialParams::setToOriginImpl)
      .def("oplus_impl", &VertexOdomDifferentialParams::oplusImpl);
}

}  // namespace g2o
