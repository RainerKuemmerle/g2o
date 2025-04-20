#pragma once

#include "detail/registry.h"
#include "g2o/types/slam3d/se3quat.h"

namespace g2o {

inline void declareSE3Quat(detail::Registry& registry) {
  py::class_<SE3Quat>(registry.mod(), "SE3Quat")
      .def(py::init<>())
      .def(py::init<const Eigen::Ref<const Matrix3>&,
                    const Eigen::Ref<const Vector3>&>(),
           "R"_a, "t"_a)
      .def(py::init<const Eigen::Quaterniond&,
                    const Eigen::Ref<const Vector3>&>(),
           "q"_a, "t"_a)
      .def(py::init<const Eigen::Ref<const Vector6>&>(), "v"_a)
      .def(py::init<const Eigen::Ref<const Vector7>&>(), "v"_a)

      .def("translation", &SE3Quat::translation)
      .def("position", &SE3Quat::translation)
      .def("set_translation", &SE3Quat::setTranslation, "t"_a)
      .def("rotation", &SE3Quat::rotation)
      .def("orientation", &SE3Quat::rotation)
      .def("Quaternion", &SE3Quat::rotation)
      .def("set_rotation", &SE3Quat::setRotation, "q"_a)

      .def(py::self * py::self)
      .def(py::self *= py::self)
      .def(py::self * Vector3())

      .def("inverse", &SE3Quat::inverse)
      .def("__getitem__", &SE3Quat::operator[], "i"_a)

      .def("to_vector", &SE3Quat::toVector)
      .def("vector", &SE3Quat::toVector)
      .def("from_vector", &SE3Quat::fromVector, "v"_a)
      .def("to_minimal_vector", &SE3Quat::toMinimalVector)
      .def("from_minimal_vector", &SE3Quat::fromMinimalVector, "v"_a)

      .def("log", &SE3Quat::log)                     // -> Vector6
      .def("map", &SE3Quat::map, "xyz"_a)            // Vector3 -> vector3D
      .def_static("exp", &SE3Quat::exp, "update"_a)  // Vector6 -> SE3Quat

      .def("adj", &SE3Quat::adj)
      .def("to_homogeneous_matrix", &SE3Quat::to_homogeneous_matrix)
      .def("matrix", &SE3Quat::to_homogeneous_matrix)
      .def("normalize_rotation", &SE3Quat::normalizeRotation)

      // operator Isometry3() const
      .def("to_isometry3d", [](const SE3Quat& p) {
        Isometry3 result(p.rotation());
        result.translation() = p.translation();
        return result;
      });
}

}  // end namespace g2o
