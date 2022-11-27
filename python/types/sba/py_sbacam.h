#pragma once

#include "g2o/types/sba/sbacam.h"
#include "g2opy.h"

namespace g2o {

inline void declareSBACam(py::module& m) {
  py::class_<SBACam, SE3Quat>(m, "SBACam")
      .def(py::init<>())
      .def(py::init<const Eigen::Quaterniond&, const Eigen::Vector3d&>(), "r"_a,
           "t"_a)
      .def(py::init<const SE3Quat&>(), "p"_a)

      .def("update", &SBACam::update, "update"_a,
           "update from the linear solution defined in se3quat")  // Vector6& ->

      .def_static("transform_w2f", &SBACam::transformW2F, "m"_a, "trans"_a,
                  "qrot"_a)  // (Eigen::Matrix<double,3,4>&, const
                             // Eigen::Vector3d&, const Eigen::Quaterniond&) ->
      .def_static("transform_f2w", &SBACam::transformF2W, "m"_a, "trans"_a,
                  "qrot"_a)  // (Eigen::Matrix<double,3,4>&, const
                             // Eigen::Vector3d&, const Eigen::Quaterniond&) ->

      .def("set_cam", &SBACam::setKcam, "fx"_a, "fy"_a, "cx"_a, "cy"_a, "tx"_a,
           "set up camera matrix")

      .def("set_transform", &SBACam::setTransform,
           "set transform from world to cam coords")  // () -> void

      .def("set_projection", &SBACam::setProjection,
           "Set up world-to-image projection matrix (w2i), assumes camera "
           "parameters are "
           "filled.")  // () -> void

      .def("set_derivative", &SBACam::setDr,
           "sets angle derivatives")  // () -> void
      .def("set_dr", &SBACam::setDr,
           "sets angle derivatives")  // () -> void

      .def_readwrite("cam", &SBACam::Kcam, "camera matrix")
      .def_readwrite("Kcam", &SBACam::Kcam, "camera matrix")
      .def_readwrite("baseline", &SBACam::baseline, "stereo baseline")

      .def_readwrite("w2n", &SBACam::w2n,
                     "transform from world to node coordinates")
      .def_readwrite("w2i", &SBACam::w2i,
                     "transform from world to image coordinates")

      // Derivatives of the rotation matrix transpose wrt quaternion xyz, used
      // for calculating Jacobian wrt pose of a projection.
      .def_readwrite("dRdx", &SBACam::dRdx)
      .def_readwrite("dRdy", &SBACam::dRdy)
      .def_readwrite("dRdz", &SBACam::dRdz);
}

}  // end namespace g2o
