#pragma once

#include <g2o/types/sclam2d/odometry_measurement.h>

#include "g2opy.h"
#include "python/core/py_base_edge.h"
#include "python/core/py_base_variable_sized_edge.h"

namespace g2o {

inline void declareOdometryMeasurement(py::module& m) {
  py::class_<VelocityMeasurement>(m, "VelocityMeasurement")
      .def(py::init<>())
      .def(py::init<double, double, double>(), "vl"_a, "vr"_a, "dt"_a)

      .def("vl", &VelocityMeasurement::vl)
      .def("set_vl", &VelocityMeasurement::setVl)
      .def("vr", &VelocityMeasurement::vr)
      .def("set_vr", &VelocityMeasurement::setVr)
      .def("dt", &VelocityMeasurement::dt)
      .def("set_dt", &VelocityMeasurement::setDt)
      .def("measurement", &VelocityMeasurement::measurement);

  py::class_<MotionMeasurement>(m, "MotionMeasurement")
      .def(py::init<>())
      .def(py::init<double, double, double, double>(), "x"_a, "y"_a, "theta"_a,
           "dt"_a)
      .def(py::init<const Vector3&, double>(), "m"_a, "dt"_a)

      .def("x", &MotionMeasurement::x)
      .def("set_x", &MotionMeasurement::setX)
      .def("y", &MotionMeasurement::y)
      .def("set_y", &MotionMeasurement::setY)
      .def("theta", &MotionMeasurement::theta)
      .def("set_theta", &MotionMeasurement::setTheta)
      .def("dt", &MotionMeasurement::dt)
      .def("set_dt", &MotionMeasurement::setDt)
      .def("measurement", &MotionMeasurement::measurement);

  py::class_<OdomConvert>(m, "OdomConvert")
      .def_static("convert_to_velocity", &OdomConvert::convertToVelocity)
      .def_static("convert_to_motion", &OdomConvert::convertToMotion)
      .def_static("to_velocity", &OdomConvert::convertToVelocity)
      .def_static("to_motion", &OdomConvert::convertToMotion)

      ;

  templatedBaseEdge<3, VelocityMeasurement>(m, "_3_VelocityMeasurement");
  templatedBaseVariableSizedEdge<3, VelocityMeasurement>(
      m, "_3_VelocityMeasurement");
}

}  // namespace g2o
