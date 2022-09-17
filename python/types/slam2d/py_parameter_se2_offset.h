#pragma once

#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2opy.h"

namespace g2o {

inline void declareParameterSE2Offset(py::module& m) {
  py::class_<ParameterSE2Offset, Parameter,
             std::shared_ptr<ParameterSE2Offset>>(m, "ParameterSE2Offset")
      .def(py::init<>())
      .def("set_offset", &ParameterSE2Offset::setOffset)
      .def("offset", &ParameterSE2Offset::offset)
      .def("offset_matrix", &ParameterSE2Offset::offsetMatrix)
      .def("inverse_offset_matrix", &ParameterSE2Offset::inverseOffsetMatrix);

  // class G2O_TYPES_SLAM2D_API CacheSE2Offset: public Cache
}

}  // namespace g2o
