#pragma once

#include "detail/registry.h"
#include "g2o/types/slam3d/parameter_camera.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2opy.h"

namespace g2o {

inline void declareSalm3dParameter(detail::Registry& registry) {
  py::class_<ParameterSE3Offset, Parameter,
             std::shared_ptr<ParameterSE3Offset>>(registry.mod(),
                                                  "ParameterSE3Offset")
      .def(py::init<>())

      .def("set_param", &ParameterSE3Offset::setParam)
      .def("set_param_data", &ParameterSE3Offset::setParameterData)
      .def("param", &ParameterSE3Offset::param)
      .def("inverse_offset", &ParameterSE3Offset::inverseOffset);

  // class G2O_TYPES_SLAM3D_API CacheSE3Offset: public Cache
  // class G2O_TYPES_SLAM3D_API CacheSE3OffsetDrawAction: public DrawAction

  py::class_<ParameterCamera, Parameter, std::shared_ptr<ParameterCamera>>(
      registry.mod(), "ParameterCamera")
      .def(py::init<>())
      .def("set_param", &ParameterCamera::setParam)
      .def("param", &ParameterCamera::param);

  // class G2O_TYPES_SLAM3D_API CacheCamera: public CacheSE3Offset
  // class G2O_TYPES_SLAM3D_API CacheCameraDrawAction: public DrawAction
}

}  // namespace g2o
