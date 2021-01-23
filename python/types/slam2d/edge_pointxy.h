#include <g2o/types/slam2d/edge_pointxy.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "python/core/base_binary_edge.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareEdgePointXY(py::module& m) {
  templatedBaseBinaryEdge<2, Vector2, VertexPointXY, VertexPointXY>(
      m, "_2_Vector2_VertexPointXY_VertexPointXY");

  py::class_<EdgePointXY, BaseBinaryEdge<2, Vector2, VertexPointXY, VertexPointXY>>(m,
                                                                                    "EdgePointXY")
      .def(py::init<>())
      .def("compute_error", &EdgePointXY::computeError)
      .def("set_measurement", &EdgePointXY::setMeasurement)
      .def("set_measurement_data", &EdgePointXY::setMeasurementData)
      .def("get_measurement_data", &EdgePointXY::getMeasurementData)
      .def("measurement_dimension", &EdgePointXY::measurementDimension)
      .def("set_measurement_from_state", &EdgePointXY::setMeasurementFromState)
      .def("initial_estimate_possible", &EdgePointXY::initialEstimatePossible)
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      .def("linearize_oplus", &EdgePointXY::linearizeOplus)
#endif
      ;
}

}  // end namespace g2o
