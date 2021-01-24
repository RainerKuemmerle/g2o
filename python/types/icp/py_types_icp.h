#include <g2o/types/icp/types_icp.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "python/core/py_base_binary_edge.h"
#include "python/core/py_base_edge.h"
#include "python/core/py_base_vertex.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareTypesICP(py::module& m) {
  py::class_<EdgeGICP>(m, "EdgeGICP")
      .def(py::init<>())

      .def("make_rot0", &EdgeGICP::makeRot0, "set up rotation matrix for pos0")
      .def("make_rot1", &EdgeGICP::makeRot1, "set up rotation matrix for pos1")

      .def("prec0", &EdgeGICP::prec0, "e"_a, "returns a precision matrix for point-plane")
      .def("prec1", &EdgeGICP::prec1, "e"_a, "returns a precision matrix for point-plane")

      .def("cov0", &EdgeGICP::cov0, "e"_a, "return a covariance matrix for plane-plane")
      .def("cov1", &EdgeGICP::cov1, "e"_a, "return a covariance matrix for plane-plane")

      // point positions
      .def_readwrite("pos0", &EdgeGICP::pos0)
      .def_readwrite("pos1", &EdgeGICP::pos1)
      // unit normals
      .def_readwrite("normal0", &EdgeGICP::normal0)
      .def_readwrite("normal1", &EdgeGICP::normal1)
      // rotation matrix for normal
      .def_readwrite("R0", &EdgeGICP::R0)
      .def_readwrite("R1", &EdgeGICP::R1);

  templatedBaseEdge<3, EdgeGICP>(m, "_3_EdgeGICP");
  templatedBaseBinaryEdge<3, EdgeGICP, VertexSE3, VertexSE3>(m, "_3_EdgeGICP_VertexSE3_VertexSE3");
  py::class_<Edge_V_V_GICP, BaseBinaryEdge<3, EdgeGICP, VertexSE3, VertexSE3>>(m, "Edge_V_V_GICP")
      .def(py::init<>())
      .def(py::init<const Edge_V_V_GICP*>())

      .def("compute_error", &Edge_V_V_GICP::computeError)
#ifdef GICP_ANALYTIC_JACOBIANS
      .def("linearize_oplus", &Edge_V_V_GICP::linearizeOplus)
#endif

      .def_readwrite("pl_pl", &Edge_V_V_GICP::pl_pl)
      .def_readwrite("cov0", &Edge_V_V_GICP::cov0)
      .def_readwrite("cov1", &Edge_V_V_GICP::cov1)

      .def_readwrite_static("dRidx", &Edge_V_V_GICP::dRidx)
      .def_readwrite_static("dRidy", &Edge_V_V_GICP::dRidy)
      .def_readwrite_static("dRidz", &Edge_V_V_GICP::dRidz);

  py::class_<VertexSCam, VertexSE3>(m, "VertexSCam")
      .def(py::init<>())

      .def("oplus_impl", &VertexSCam::oplusImpl)

      .def_static("transform_w2f", &VertexSCam::transformW2F, "m"_a, "trans"_a,
                  "qrot"_a)  // (Eigen::Matrix<double,3,4>&, const Eigen::Vector3d&, const
                             // Eigen::Quaterniond&) ->
      .def_static("transform_f2w", &VertexSCam::transformF2W, "m"_a, "trans"_a,
                  "qrot"_a)  // (Eigen::Matrix<double,3,4>&, const Eigen::Vector3d&, const
                             // Eigen::Quaterniond&) ->

      .def_static("set_cam", &VertexSCam::setKcam, "fx"_a, "fy"_a, "cx"_a, "cy"_a, "tx"_a,
                  "set up camera matrix")

      .def("set_transform", &VertexSCam::setTransform,
           "set transform from world to cam coords")  // () -> void

      .def("set_projection", &VertexSCam::setProjection,
           "Set up world-to-image projection matrix (w2i), assumes camera parameters are "
           "filled.")  // () -> void

      .def("set_derivative", &VertexSCam::setDr,
           "sets angle derivatives")  // () -> void
      .def("set_dr", &VertexSCam::setDr, "sets angle derivatives")

      .def("set_all", &VertexSCam::setAll, "set all aux transforms")

      //.def("map_point", &VertexSCam::mapPoint,
      //        "res"_a, "pt3"_a,
      //        "calculate stereo projection")   // (Vector3&, const Vector3&) ->
      .def("map_point",
           [](VertexSCam& cam, const Vector3& point) {
             Vector3 res;
             cam.mapPoint(res, point);
             return res;
           })

      // camera matrix and stereo baseline
      .def_readwrite_static("Kcam", &VertexSCam::Kcam)
      .def_readwrite_static("baseline", &VertexSCam::baseline)

      // transformations
      .def_readwrite("w2n", &VertexSCam::w2n)  // transform from world to node coordinates
      .def_readwrite("w2i", &VertexSCam::w2i)  // transform from world to image coordinates

      // Derivatives of the rotation matrix transpose wrt quaternion xyz, used for
      // calculating Jacobian wrt pose of a projection.
      .def_readwrite("dRdx", &VertexSCam::dRdx)
      .def_readwrite("dRdy", &VertexSCam::dRdy)
      .def_readwrite("dRdz", &VertexSCam::dRdz)

      .def_readwrite_static("dRidx", &VertexSCam::dRidx)
      .def_readwrite_static("dRidy", &VertexSCam::dRidy)
      .def_readwrite_static("dRidz", &VertexSCam::dRidz);

  templatedBaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexSCam>(
      m, "_3_Vector3_VertexSBAPointXYZ_VertexSCam");
  py::class_<Edge_XYZ_VSC, BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexSCam>>(m,
                                                                                   "Edge_XYZ_VSC")
      .def(py::init<>())
      .def("compute_error", &Edge_XYZ_VSC::computeError)

#ifdef SCAM_ANALYTIC_JACOBIANS
      .def("linearize_oplus", &Edge_XYZ_VSC::linearizeOplus)
#endif

      ;
}

}  // namespace g2o
