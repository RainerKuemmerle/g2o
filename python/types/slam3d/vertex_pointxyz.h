#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareVertexPointXYZ(py::module& m) {
  py::class_<VertexPointXYZ, BaseVertex<3, Vector3>>(m, "VertexPointXYZ")
      .def(py::init<>())

      .def("set_to_origin_impl", &VertexPointXYZ::setToOriginImpl)
      .def("set_estimate_data_impl", &VertexPointXYZ::setEstimateDataImpl)
      .def("get_estimate_data", &VertexPointXYZ::getEstimateData)
      .def("estimate_dimension", &VertexPointXYZ::estimateDimension)
      .def("set_minimal_estimate_data_impl", &VertexPointXYZ::setMinimalEstimateDataImpl)
      .def("get_minimal_estimate_data", &VertexPointXYZ::getMinimalEstimateData)
      .def("minimal_estimate_dimension", &VertexPointXYZ::minimalEstimateDimension)
      .def("oplus_impl", &VertexPointXYZ::oplusImpl);

  // class G2O_TYPES_SLAM3D_API VertexPointXYZWriteGnuplotAction: public WriteGnuplotAction
  // class VertexPointXYZDrawAction: public DrawAction
}

}  // end namespace g2o