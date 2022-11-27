#include "py_types_sba.h"

#include "g2o/types/sba/types_sba.h"
#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"
#include "python/core/py_base_vertex.h"

namespace g2o {

void declareTypesSBA(py::module& m) {
  py::class_<VertexIntrinsics,
             BaseVertex<4, Eigen::Matrix<double, 5, 1, Eigen::ColMajor>>,
             std::shared_ptr<VertexIntrinsics>>(m, "VertexIntrinsics")
      .def(py::init<>())
      .def("set_to_origin_impl", &VertexIntrinsics::setToOriginImpl)
      ;

  templatedBaseVertex<6, SBACam>(m, "_6_SBACam");
  py::class_<VertexCam, BaseVertex<6, SBACam>, std::shared_ptr<VertexCam>>(
      m, "VertexCam")
      .def(py::init<>())
      .def("set_to_origin_impl", &VertexCam::setToOriginImpl)
      .def("set_estimate", &VertexCam::setEstimate)  // const SBACam& -> void
      .def("set_estimate_data_impl", &VertexCam::setEstimateDataImpl)
      .def("get_estimate_data", &VertexCam::getEstimateData)
      .def("estimate_dimension", &VertexCam::estimateDimension)
      .def("set_minimal_estimate_data_impl",
           &VertexCam::setMinimalEstimateDataImpl)
      .def("get_minimal_estimate_data", &VertexCam::getMinimalEstimateData)
      .def("minimal_estimate_dimension", &VertexCam::minimalEstimateDimension);

  // monocular projection
  // first two args are the measurement type, second two the connection classes
  templatedBaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexCam>(
      m, "_2_Vector2_VertexPointXYZ_VertexCam");
  py::class_<EdgeProjectP2MC,
             BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexCam>,
             std::shared_ptr<EdgeProjectP2MC>>(m, "EdgeProjectP2MC")
      .def(py::init<>())
      .def("compute_error", &EdgeProjectP2MC::computeError)  // () -> void
      .def("linearize_oplus", &EdgeProjectP2MC::linearizeOplus);

  // stereo projection
  // first two args are the measurement type, second two the connection classes
  templatedBaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexCam>(
      m, "_3_Vector3_VertexPointXYZ_VertexCam");
  py::class_<EdgeProjectP2SC,
             BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexCam>,
             std::shared_ptr<EdgeProjectP2SC>>(m, "EdgeProjectP2SC")
      .def(py::init<>())
      .def("compute_error", &EdgeProjectP2SC::computeError)  // () -> void
      .def("linearize_oplus", &EdgeProjectP2SC::linearizeOplus);

  templatedBaseBinaryEdge<6, SE3Quat, VertexCam, VertexCam>(
      m, "_6_SE3Quat_VertexCam_VertexCam");
  py::class_<EdgeSBACam, BaseBinaryEdge<6, SE3Quat, VertexCam, VertexCam>,
             std::shared_ptr<EdgeSBACam>>(m, "EdgeSBACam")
      .def(py::init<>())
      .def("compute_error", &EdgeSBACam::computeError)
      .def("set_measurement", &EdgeSBACam::setMeasurement)
      .def("initial_estimate_possible", &EdgeSBACam::initialEstimatePossible)
      .def("set_measurement_data", &EdgeSBACam::setMeasurementData)
      .def("get_measurement_data", &EdgeSBACam::getMeasurementData)
      .def("measurement_dimension", &EdgeSBACam::measurementDimension)
      .def("set_measurement_from_state", &EdgeSBACam::setMeasurementFromState);

  templatedBaseBinaryEdge<1, double, VertexCam, VertexCam>(
      m, "_1_double_VertexCam_VertexCam");
  py::class_<EdgeSBAScale, BaseBinaryEdge<1, double, VertexCam, VertexCam>,
             std::shared_ptr<EdgeSBAScale>>(m, "EdgeSBAScale")
      .def(py::init<>())
      .def("compute_error", &EdgeSBAScale::computeError)
      .def("set_measurement", &EdgeSBAScale::setMeasurement)
      .def("initial_estimate_possible", &EdgeSBAScale::initialEstimatePossible)
      .def("initial_estimate", &EdgeSBAScale::initialEstimate);
}

}  // end namespace g2o
