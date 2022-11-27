#include "py_types_seven_dof_expmap.h"

#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2opy.h"
#include "py_sim3.h"
#include "python/core/py_base_binary_edge.h"

namespace g2o {

void declareTypesSevenDofExpmap(py::module& m) {
  declareSim3(m);

  py::class_<VertexSim3Expmap, BaseVertex<7, Sim3>,
             std::shared_ptr<VertexSim3Expmap>>(m, "VertexSim3Expmap")
      .def(py::init<>())
      .def("set_to_origin_impl", &VertexSim3Expmap::setToOriginImpl)
      .def("cam_map1", &VertexSim3Expmap::cam_map1)
      .def("cam_map2", &VertexSim3Expmap::cam_map2);

  templatedBaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>(
      m, "_7_Sim3_VertexSim3Expmap_VertexSim3Expmap");

  py::class_<EdgeSim3,
             BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>,
             std::shared_ptr<EdgeSim3>>(m, "EdgeSim3")
      .def(py::init<>())
      .def("compute_error", &EdgeSim3::computeError)
      .def("initial_estimate_possible", &EdgeSim3::initialEstimatePossible)
      .def("initial_estimate", &EdgeSim3::initialEstimate);

  templatedBaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSim3Expmap>(
      m, "_2_Vector2_VertexPointXYZ_VertexSim3Expmap");

  py::class_<EdgeSim3ProjectXYZ,
             BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSim3Expmap>,
             std::shared_ptr<EdgeSim3ProjectXYZ>>(m, "EdgeSim3ProjectXYZ")
      .def(py::init<>())
      .def("compute_error", &EdgeSim3ProjectXYZ::computeError);

  py::class_<EdgeInverseSim3ProjectXYZ,
             BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSim3Expmap>,
             std::shared_ptr<EdgeInverseSim3ProjectXYZ>>(
      m, "EdgeInverseSim3ProjectXYZ")
      .def(py::init<>())
      .def("compute_error", &EdgeInverseSim3ProjectXYZ::computeError);
}

}  // namespace g2o
