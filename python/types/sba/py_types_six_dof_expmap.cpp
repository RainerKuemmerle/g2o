#include "py_types_six_dof_expmap.h"

#include "g2o/core/factory.h"
#include "g2o/types/sba/edge_project_psi2uv.h"
#include "g2o/types/sba/edge_project_stereo_xyz.h"
#include "g2o/types/sba/edge_project_stereo_xyz_onlypose.h"
#include "g2o/types/sba/edge_project_xyz.h"
#include "g2o/types/sba/edge_project_xyz2uv.h"
#include "g2o/types/sba/edge_project_xyz2uvu.h"
#include "g2o/types/sba/edge_project_xyz_onlypose.h"
#include "g2o/types/sba/edge_se3_expmap.h"
#include "g2o/types/sba/parameter_cameraparameters.h"
#include "g2opy.h"

G2O_USE_TYPE_GROUP(expmap)

namespace g2o {

void declareTypesSixDofExpmap(detail::Registry& registry) {
  py::classh<StereoCameraParameters>(registry.mod(), "StereoCameraParameters")
      .def(py::init<>())
      .def_readwrite("focal_length", &StereoCameraParameters::focal_length)
      .def_readwrite("principle_point",
                     &StereoCameraParameters::principle_point)
      .def_readwrite("baseline", &StereoCameraParameters::baseline);

  py::classh<CameraParameters, Parameter>(registry.mod(), "CameraParameters")
      .def(py::init<>())
      .def(py::init([](double f, const Eigen::Ref<const Vector2>& p, double b) {
        return CameraParameters(f, p, b);
      }))
      .def("cam_map", &CameraParameters::cam_map, "trans_xyz"_a)
      .def("stereocam_uvu_map", &CameraParameters::stereocam_uvu_map,
           "trans_xyz"_a);

  registry.registerVertex<VertexSE3Expmap>("VertexSE3Expmap");

  registry.registerEdgeFixed<EdgeSE3Expmap>("EdgeSE3Expmap");

  registry.registerEdgeFixed<EdgeProjectXYZ2UV>("EdgeProjectXYZ2UV");

  registry.registerEdgeFixed<EdgeProjectPSI2UV>("EdgeProjectPSI2UV");

  registry.registerEdgeFixed<EdgeProjectXYZ2UVU>("EdgeProjectXYZ2UVU");

  registry.registerEdgeFixed<EdgeSE3ProjectXYZ>("EdgeSE3ProjectXYZ")
      .def("is_depth_positive", &EdgeSE3ProjectXYZ::isDepthPositive)
      .def("cam_project", &EdgeSE3ProjectXYZ::cam_project)
      .def_readwrite("fx", &EdgeSE3ProjectXYZ::fx)
      .def_readwrite("fy", &EdgeSE3ProjectXYZ::fy)
      .def_readwrite("cx", &EdgeSE3ProjectXYZ::cx)
      .def_readwrite("cy", &EdgeSE3ProjectXYZ::cy);

  registry
      .registerEdgeFixed<EdgeSE3ProjectXYZOnlyPose>("EdgeSE3ProjectXYZOnlyPose")
      .def("is_depth_positive", &EdgeSE3ProjectXYZOnlyPose::isDepthPositive)
      .def("cam_project", &EdgeSE3ProjectXYZOnlyPose::cam_project)
      .def_readwrite("fx", &EdgeSE3ProjectXYZOnlyPose::fx)
      .def_readwrite("fy", &EdgeSE3ProjectXYZOnlyPose::fy)
      .def_readwrite("cx", &EdgeSE3ProjectXYZOnlyPose::cx)
      .def_readwrite("cy", &EdgeSE3ProjectXYZOnlyPose::cy)
      .def_readwrite("Xw", &EdgeSE3ProjectXYZOnlyPose::Xw);

  registry.registerEdgeFixed<EdgeStereoSE3ProjectXYZ>("EdgeStereoSE3ProjectXYZ")
      .def("is_depth_positive", &EdgeStereoSE3ProjectXYZ::isDepthPositive)
      .def("cam_project", &EdgeStereoSE3ProjectXYZ::cam_project)
      .def_readwrite("fx", &EdgeStereoSE3ProjectXYZ::fx)
      .def_readwrite("fy", &EdgeStereoSE3ProjectXYZ::fy)
      .def_readwrite("cx", &EdgeStereoSE3ProjectXYZ::cx)
      .def_readwrite("cy", &EdgeStereoSE3ProjectXYZ::cy)
      .def_readwrite("bf", &EdgeStereoSE3ProjectXYZ::bf);

  registry
      .registerEdgeFixed<EdgeStereoSE3ProjectXYZOnlyPose>(
          "EdgeStereoSE3ProjectXYZOnlyPose")
      .def("is_depth_positive",
           &EdgeStereoSE3ProjectXYZOnlyPose::isDepthPositive)
      .def("cam_project", &EdgeStereoSE3ProjectXYZOnlyPose::cam_project);
}

}  // end namespace g2o
