#pragma once

#include <g2o/types/sclam2d/odometry_measurement.h>

#include "detail/registry.h"
#include "g2opy.h"

namespace g2o {

inline void declareOdometryMeasurement(detail::Registry& registry) {
  py::classh<VelocityMeasurement>(registry.mod(), "VelocityMeasurement")
      .def(py::init<>())
      .def(py::init<double, double, double>(), "vl"_a, "vr"_a, "dt"_a)

      .def("vl", &VelocityMeasurement::vl)
      .def("set_vl", &VelocityMeasurement::setVl)
      .def("vr", &VelocityMeasurement::vr)
      .def("set_vr", &VelocityMeasurement::setVr)
      .def("dt", &VelocityMeasurement::dt)
      .def("set_dt", &VelocityMeasurement::setDt)
      .def("measurement", &VelocityMeasurement::measurement);

  py::classh<MotionMeasurement>(registry.mod(), "MotionMeasurement")
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

  py::classh<OdomConvert>(registry.mod(), "OdomConvert")
      .def_static("convert_to_velocity", &OdomConvert::convertToVelocity)
      .def_static("convert_to_motion", &OdomConvert::convertToMotion)
      .def_static("to_velocity", &OdomConvert::convertToVelocity)
      .def_static("to_motion", &OdomConvert::convertToMotion);
}

}  // namespace g2o
