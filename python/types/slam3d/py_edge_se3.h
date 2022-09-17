#pragma once

#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_lotsofxyz.h"
#include "g2o/types/slam3d/edge_se3_offset.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"
#include "python/core/py_base_unary_edge.h"

namespace g2o {

inline void declareEdgeSE3(py::module& m) {
  templatedBaseBinaryEdge<6, Isometry3, VertexSE3, VertexSE3>(
      m, "_6_Isometry3_VertexSE3_VertexSE3");
  py::class_<EdgeSE3, BaseBinaryEdge<6, Isometry3, VertexSE3, VertexSE3>,
             std::shared_ptr<EdgeSE3>>(m, "EdgeSE3")
      .def(py::init<>())

      .def("compute_error", &EdgeSE3::computeError)
      .def("set_measurement", &EdgeSE3::setMeasurement)
      .def("set_measurement_data", &EdgeSE3::setMeasurementData)
      .def("get_measurement_data", &EdgeSE3::getMeasurementData)
      .def("measurement_dimension", &EdgeSE3::measurementDimension)
      .def("set_measurement_from_state", &EdgeSE3::setMeasurementFromState)
      .def("initial_estimate_possible", &EdgeSE3::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE3::initialEstimate)
      .def("linearize_oplus", &EdgeSE3::linearizeOplus);

  // class G2O_TYPES_SLAM3D_API EdgeSE3WriteGnuplotAction: public
  // WriteGnuplotAction class G2O_TYPES_SLAM3D_API EdgeSE3DrawAction: public
  // DrawAction

  py::class_<EdgeSE3LotsOfXYZ, BaseVariableSizedEdge<-1, VectorX>,
             std::shared_ptr<EdgeSE3LotsOfXYZ>>(m, "EdgeSE3LotsOfXYZ")
      .def(py::init<>())

      .def("set_dimension", &EdgeSE3LotsOfXYZ::setDimension<-1>)
      .def("set_size", &EdgeSE3LotsOfXYZ::setSize)
      .def("compute_error", &EdgeSE3LotsOfXYZ::computeError)
      .def("set_measurement_from_state",
           &EdgeSE3LotsOfXYZ::setMeasurementFromState)
      .def("initial_estimate", &EdgeSE3LotsOfXYZ::initialEstimate)
      .def("initial_estimate_possible",
           &EdgeSE3LotsOfXYZ::initialEstimatePossible)
      .def("linearize_oplus", &EdgeSE3LotsOfXYZ::linearizeOplus);

  py::class_<EdgeSE3Offset, EdgeSE3, std::shared_ptr<EdgeSE3Offset>>(
      m, "EdgeSE3Offset")
      .def(py::init<>())

      .def("compute_error", &EdgeSE3Offset::computeError)
      .def("set_measurement_from_state",
           &EdgeSE3Offset::setMeasurementFromState)
      .def("initial_estimate", &EdgeSE3Offset::initialEstimate)
      .def("initial_estimate_possible", &EdgeSE3Offset::initialEstimatePossible)
      .def("linearize_oplus", &EdgeSE3Offset::linearizeOplus);

  templatedBaseUnaryEdge<6, Isometry3, VertexSE3>(m, "_6_Isometry3_VertexSE3");
  py::class_<EdgeSE3Prior, BaseUnaryEdge<6, Isometry3, VertexSE3>,
             std::shared_ptr<EdgeSE3Prior>>(m, "EdgeSE3Prior")
      .def(py::init<>())

      .def("compute_error", &EdgeSE3Prior::computeError)
      .def("set_measurement", &EdgeSE3Prior::setMeasurement)
      .def("set_measurement_data", &EdgeSE3Prior::setMeasurementData)
      .def("get_measurement_data", &EdgeSE3Prior::getMeasurementData)
      .def("measurement_dimension", &EdgeSE3Prior::measurementDimension)
      .def("set_measurement_from_state", &EdgeSE3Prior::setMeasurementFromState)
      .def("initial_estimate_possible", &EdgeSE3Prior::initialEstimatePossible)
      .def("initial_estimate", &EdgeSE3Prior::initialEstimate)
      .def("linearize_oplus", &EdgeSE3Prior::linearizeOplus);
}

}  // namespace g2o
