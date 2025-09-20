#pragma once

#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2opy.h"

namespace g2o {

inline void declareParameterSE2Offset(py::module& m) {
  py::classh<ParameterSE2Offset, Parameter>(m, "ParameterSE2Offset")
      .def(py::init<>())
      .def("set_param", &ParameterSE2Offset::setParam)
      .def("set_param_data", &ParameterSE2Offset::setParameterData)
      .def("param", &ParameterSE2Offset::param)
      .def("offset_matrix", &ParameterSE2Offset::offsetMatrix)
      .def("inverse_offset_matrix", &ParameterSE2Offset::inverseOffsetMatrix);

  // class G2O_TYPES_SLAM2D_API CacheSE2Offset: public Cache
}

}  // namespace g2o
