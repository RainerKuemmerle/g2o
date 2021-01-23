#include <g2o/core/parameter.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareParameter(py::module& m) {
  py::class_<Parameter, HyperGraph::HyperGraphElement>(m, "Parameter")
      //.def(py::init<>())
      .def("id", &Parameter::id)
      .def("set_id", &Parameter::setId, "id"_a)
      .def("element_type", &Parameter::elementType);
}

}  // end namespace g2o