#pragma once

#include <g2o/core/parameter.h>
#include <g2o/core/parameter_container.h>

#include "g2opy.h"

namespace g2o {

inline void declareParameter(py::module& m) {
  py::classh<Parameter, HyperGraph::HyperGraphElement>(m, "Parameter")
      //.def(py::init<>())
      .def("id", &Parameter::id)
      .def("set_id", &Parameter::setId, "id"_a)
      .def("element_type", &Parameter::elementType);
}

inline void declareParameterContainer(py::module& m) {
  py::classh<ParameterContainer>(m, "ParameterContainer")
      .def(py::init<>())
      .def("add_parameter", &ParameterContainer::addParameter, "parameter"_a,
           py::keep_alive<1, 2>())
      .def("get_parameter", &ParameterContainer::getParameter, "id"_a)
      .def("detach_parameter", &ParameterContainer::detachParameter, "id"_a)
      .def("clear", [](ParameterContainer& self) { self.clear(); })
      .def("size", [](const ParameterContainer& self) { return self.size(); })
      .def("__len__", [](const ParameterContainer& self) {
        return static_cast<py::ssize_t>(self.size());
      });
}

}  // end namespace g2o
