#pragma once

#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_lotsofxy.h"
#include "g2o/types/slam2d/edge_se2_offset.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/edge_se2_twopointsxy.h"
#include "g2o/types/slam2d/edge_se2_xyprior.h"
#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"
#include "python/core/py_base_unary_edge.h"

namespace g2o {

inline void declareEdgeSE2(py::module& m) {
  templatedBaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>(
      m, "_3_SE2_VertexSE2_VertexSE2");
  py::class_<EdgeSE2, BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>,
             std::shared_ptr<EdgeSE2>>(m, "EdgeSE2")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2::computeError)
      .def("set_measurement", &EdgeSE2::setMeasurement)
      .def("set_measurement_data", &EdgeSE2::setMeasurementData)
      .def("get_measurement_data", &EdgeSE2::getMeasurementData)
      .def("measurement_dimension", &EdgeSE2::measurementDimension)
      .def("set_measurement_from_state", &EdgeSE2::setMeasurementFromState)
      .def("initial_estimate_possible", &EdgeSE2::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2::initialEstimate)
      .def("linearize_oplus", &EdgeSE2::linearizeOplus);

  py::class_<EdgeSE2LotsOfXY, BaseVariableSizedEdge<-1, VectorX>,
             std::shared_ptr<EdgeSE2LotsOfXY>>(m, "EdgeSE2LotsOfXY")
      .def(py::init<>())
      .def("set_dimension", &EdgeSE2LotsOfXY::setDimension<-1>)
      .def("set_size", &EdgeSE2LotsOfXY::setSize)

      .def("compute_error", &EdgeSE2LotsOfXY::computeError)
      .def("set_measurement_from_state",
           &EdgeSE2LotsOfXY::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE2LotsOfXY::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2LotsOfXY::initialEstimate)
      .def("linearize_oplus", &EdgeSE2LotsOfXY::linearizeOplus);

  py::class_<EdgeSE2Offset, BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>,
             std::shared_ptr<EdgeSE2Offset>>(m, "EdgeSE2Offset")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2Offset::computeError)
      .def("set_measurement", &EdgeSE2Offset::setMeasurement)
      .def("set_measurement_data", &EdgeSE2Offset::setMeasurementData)
      .def("get_measurement_data", &EdgeSE2Offset::getMeasurementData)
      .def("measurement_dimension", &EdgeSE2Offset::measurementDimension)
      .def("set_measurement_from_state",
           &EdgeSE2Offset::setMeasurementFromState)
      .def("initial_estimate_possible", &EdgeSE2Offset::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2Offset::initialEstimate);

  templatedBaseUnaryEdge<3, SE2, VertexSE2>(m, "_3_SE2_VertexSE2");
  py::class_<EdgeSE2Prior, BaseUnaryEdge<3, SE2, VertexSE2>,
             std::shared_ptr<EdgeSE2Prior>>(m, "EdgeSE2Prior")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2Prior::computeError)
      .def("set_measurement", &EdgeSE2Prior::setMeasurement)
      .def("set_measurement_data", &EdgeSE2Prior::setMeasurementData)
      .def("get_measurement_data", &EdgeSE2Prior::getMeasurementData)
      .def("measurement_dimension", &EdgeSE2Prior::measurementDimension)
      .def("initial_estimate_possible", &EdgeSE2Prior::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2Prior::initialEstimate);

  py::class_<EdgeSE2TwoPointsXY, BaseVariableSizedEdge<4, Vector4>,
             std::shared_ptr<EdgeSE2TwoPointsXY>>(m, "EdgeSE2TwoPointsXY")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2TwoPointsXY::computeError)
      .def("set_measurement_from_state",
           &EdgeSE2TwoPointsXY::setMeasurementFromState)
      .def("initial_estimate_possible",
           &EdgeSE2TwoPointsXY::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE2TwoPointsXY::initialEstimate);

  templatedBaseUnaryEdge<2, Vector2, VertexSE2>(m, "_2_Vector2_VertexSE2");
  py::class_<EdgeSE2XYPrior, BaseUnaryEdge<2, Vector2, VertexSE2>,
             std::shared_ptr<EdgeSE2XYPrior>>(m, "EdgeSE2XYPrior")
      .def(py::init<>())
      .def("compute_error", &EdgeSE2XYPrior::computeError)
      .def("set_measurement_data", &EdgeSE2XYPrior::setMeasurementData)
      .def("get_measurement_data", &EdgeSE2XYPrior::getMeasurementData)
      .def("measurement_dimension", &EdgeSE2XYPrior::measurementDimension)
      .def("linearize_oplus", &EdgeSE2XYPrior::linearizeOplus);
}

}  // end namespace g2o
