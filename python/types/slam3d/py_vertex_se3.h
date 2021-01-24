#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "g2o/types/slam3d/vertex_se3.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareVertexSE3(py::module& m) {
  py::class_<VertexSE3, BaseVertex<6, Isometry3>>(m, "VertexSE3")
      .def(py::init<>())

      .def("set_to_origin_impl", &VertexSE3::setToOriginImpl)
      .def("set_estimate_data_impl", &VertexSE3::setEstimateDataImpl)
      .def("get_estimate_data", &VertexSE3::getEstimateData)
      .def("estimate_dimension", &VertexSE3::estimateDimension)
      .def("set_minimal_estimate_data_impl", &VertexSE3::setMinimalEstimateDataImpl)
      .def("get_minimal_estimate_data", &VertexSE3::getMinimalEstimateData)
      .def("minimal_estimate_dimension", &VertexSE3::minimalEstimateDimension)
      .def("oplus_impl", &VertexSE3::oplusImpl);

  /*
  py::class_<VertexSE3WriteGnuplotAction, WriteGnuplotAction>(m, "VertexSE3WriteGnuplotAction")
      .def(py::init<>())
      .def("__call__", &VertexSE3WriteGnuplotAction::operator())
  ;

  // #ifdef G2O_HAVE_OPENGL
  py::class_<VertexSE3DrawAction, DrawAction>(m, "VertexSE3DrawAction")
      .def(py::init<>())
      .def("__call__", &VertexSE3DrawAction::operator())
  ;
  // #endif
  */
}

}  // end namespace g2o
