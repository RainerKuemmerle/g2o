#include "py_types_six_dof_expmap.h"

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"
#include "python/core/py_base_fixed_sized_edge.h"
#include "python/core/py_base_unary_edge.h"
#include "python/core/py_base_variable_sized_edge.h"

namespace g2o {

void declareTypesSixDofExpmap(py::module& m) {
  py::class_<CameraParameters, Parameter, std::shared_ptr<CameraParameters>>(
      m, "CameraParameters")
      .def(py::init<>())
      .def(py::init([](double f, const Eigen::Ref<const Vector2>& p, double b) {
        return CameraParameters(f, p, b);
      }))

      .def("cam_map", &CameraParameters::cam_map, "trans_xyz"_a)
      .def("stereocam_uvu_map", &CameraParameters::stereocam_uvu_map,
           "trans_xyz"_a)
      .def_readwrite("focal_length", &CameraParameters::focal_length)
      .def_readwrite("principal_point", &CameraParameters::principle_point)
      .def_readwrite("principle_point", &CameraParameters::principle_point)
      .def_readwrite("baseline", &CameraParameters::baseline)
      // read
      // write
      ;

  py::class_<VertexSE3Expmap, BaseVertex<6, SE3Quat>,
             std::shared_ptr<VertexSE3Expmap>>(m, "VertexSE3Expmap")
      .def(py::init<>())
      //.def(py::init([]() {return new VertexSE3Expmap();}))
      .def("set_to_origin_impl", &VertexSE3Expmap::setToOriginImpl);

  templatedBaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>(
      m, "_6_SE3Quat_VertexSE3Expmap_VertexSE3Expmap");
  py::class_<EdgeSE3Expmap,
             BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>,
             std::shared_ptr<EdgeSE3Expmap>>(m, "EdgeSE3Expmap")
      .def(py::init<>())
      .def("compute_error", &EdgeSE3Expmap::computeError)
      .def("linearize_oplus", &EdgeSE3Expmap::linearizeOplus);

  templatedBaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSE3Expmap>(
      m, "_2_Vector2_VertexPointXYZ_VertexSE3Expmap");
  py::class_<EdgeProjectXYZ2UV,
             BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSE3Expmap>,
             std::shared_ptr<EdgeProjectXYZ2UV>>(m, "EdgeProjectXYZ2UV")
      .def(py::init<>())
      .def("compute_error", &EdgeProjectXYZ2UV::computeError)
      .def("linearize_oplus", &EdgeProjectXYZ2UV::linearizeOplus);

  templatedBaseFixedSizedEdge<2, Vector2, VertexPointXYZ, VertexSE3Expmap,
                              VertexSE3Expmap>(
      m, "2_Vector2_VertexPointXYZ_VertexSE3Expmap_VertexSE3Expmap");
  py::class_<EdgeProjectPSI2UV,
             g2o::BaseFixedSizedEdge<2, Vector2, VertexPointXYZ,
                                     VertexSE3Expmap, VertexSE3Expmap>,
             std::shared_ptr<EdgeProjectPSI2UV>>(m, "EdgeProjectPSI2UV")
      .def(py::init())
      .def("compute_error", &EdgeProjectPSI2UV::computeError)
      .def("linearize_oplus", &EdgeProjectPSI2UV::linearizeOplus);

  // Stereo Observations
  templatedBaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexSE3Expmap>(
      m, "_3_Vector3_VertexPointXYZ_VertexSE3Expmap");
  py::class_<EdgeProjectXYZ2UVU,
             BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexSE3Expmap>,
             std::shared_ptr<EdgeProjectXYZ2UVU>>(m, "EdgeProjectXYZ2UVU")
      .def(py::init<>())
      .def("compute_error", &EdgeProjectXYZ2UVU::computeError);

  // Projection using focal_length in x and y directions
  py::class_<EdgeSE3ProjectXYZ,
             BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSE3Expmap>,
             std::shared_ptr<EdgeSE3ProjectXYZ>>(m, "EdgeSE3ProjectXYZ")
      .def(py::init<>())
      .def("compute_error", &EdgeSE3ProjectXYZ::computeError)
      .def("is_depth_positive", &EdgeSE3ProjectXYZ::isDepthPositive)
      .def("linearize_oplus", &EdgeSE3ProjectXYZ::linearizeOplus)
      .def("cam_project", &EdgeSE3ProjectXYZ::cam_project);

  // Edge to optimize only the camera pose
  templatedBaseUnaryEdge<2, Vector2, VertexSE3Expmap>(
      m, "BaseUnaryEdge_2_Vector2_VertexSE3Expmap");
  py::class_<EdgeSE3ProjectXYZOnlyPose,
             BaseUnaryEdge<2, Vector2, VertexSE3Expmap>,
             std::shared_ptr<EdgeSE3ProjectXYZOnlyPose>>(
      m, "EdgeSE3ProjectXYZOnlyPose")
      .def(py::init<>())
      .def("compute_error", &EdgeSE3ProjectXYZOnlyPose::computeError)
      .def("is_depth_positive", &EdgeSE3ProjectXYZOnlyPose::isDepthPositive)
      .def("linearize_oplus", &EdgeSE3ProjectXYZOnlyPose::linearizeOplus)
      .def("cam_project", &EdgeSE3ProjectXYZOnlyPose::cam_project);

  // Projection using focal_length in x and y directions stereo
  py::class_<EdgeStereoSE3ProjectXYZ,
             BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexSE3Expmap>,
             std::shared_ptr<EdgeStereoSE3ProjectXYZ>>(
      m, "EdgeStereoSE3ProjectXYZ")
      .def(py::init<>())
      .def("compute_error", &EdgeStereoSE3ProjectXYZ::computeError)
      .def("is_depth_positive", &EdgeStereoSE3ProjectXYZ::isDepthPositive)
      .def("linearize_oplus", &EdgeStereoSE3ProjectXYZ::linearizeOplus)
      .def("cam_project", &EdgeStereoSE3ProjectXYZ::cam_project);

  // Edge to optimize only the camera pose stereo
  templatedBaseUnaryEdge<3, Vector3, VertexSE3Expmap>(
      m, "BaseUnaryEdge_3_Vector3_VertexSE3Expmap");
  py::class_<EdgeStereoSE3ProjectXYZOnlyPose,
             BaseUnaryEdge<3, Vector3, VertexSE3Expmap>,
             std::shared_ptr<EdgeStereoSE3ProjectXYZOnlyPose>>(
      m, "EdgeStereoSE3ProjectXYZOnlyPose")
      .def(py::init<>())
      .def("compute_error", &EdgeStereoSE3ProjectXYZOnlyPose::computeError)
      .def("is_depth_positive",
           &EdgeStereoSE3ProjectXYZOnlyPose::isDepthPositive)
      .def("linearize_oplus", &EdgeStereoSE3ProjectXYZOnlyPose::linearizeOplus)
      .def("cam_project", &EdgeStereoSE3ProjectXYZOnlyPose::cam_project);
}

}  // end namespace g2o
