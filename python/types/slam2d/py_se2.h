#pragma once

#include "g2o/types/slam2d/se2.h"
#include "g2opy.h"
#include "python/core/py_base_edge.h"
#include "python/core/py_base_variable_sized_edge.h"
#include "python/core/py_base_vertex.h"

namespace g2o {

inline void declareSE2(py::module& m) {
  py::class_<SE2>(m, "SE2")
      .def(py::init<>())
      .def(py::init<const Isometry2&>())
      .def(py::init<const Vector3&>())
      .def(py::init<double, double, double>())

      .def("translation", &SE2::translation)
      .def("rotation", &SE2::rotation)

      .def(py::self * py::self)
      .def(py::self * Vector2())
      .def(py::self *= py::self)

      .def("inverse", &SE2::inverse)
      .def("__getitem__", &SE2::operator[])
      .def("from_vector", &SE2::fromVector)
      .def("to_vector", &SE2::toVector)
      .def("vector", &SE2::toVector)
      .def("to_isometry", &SE2::toIsometry)
      .def("Isometry2", &SE2::toIsometry);

  templatedBaseVertex<3, SE2>(m, "_3_SE2");
  templatedBaseEdge<3, SE2>(m, "_3_SE2");
  templatedBaseVariableSizedEdge<3, SE2>(m, "_3_SE2");
}

}  // namespace g2o
