#include <g2o/types/slam2d/parameter_se2_offset.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareParameterSE2Offset(py::module& m) {
  py::class_<ParameterSE2Offset, Parameter>(m, "ParameterSE2Offset")
      .def(py::init<>())
      .def("set_offset", &ParameterSE2Offset::setOffset)
      .def("offset", &ParameterSE2Offset::offset)
      .def("offset_matrix", &ParameterSE2Offset::offsetMatrix)
      .def("inverse_offset_matrix", &ParameterSE2Offset::inverseOffsetMatrix);

  // class G2O_TYPES_SLAM2D_API CacheSE2Offset: public Cache
}

}  // namespace g2o