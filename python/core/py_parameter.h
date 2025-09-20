#pragma once

#include <g2o/core/parameter.h>

#include "g2opy.h"

namespace g2o {

inline void declareParameter(py::module& m) {
  py::classh<Parameter, HyperGraph::HyperGraphElement>(m, "Parameter")
      //.def(py::init<>())
      .def("id", &Parameter::id)
      .def("set_id", &Parameter::setId, "id"_a)
      .def("element_type", &Parameter::elementType);
}

}  // end namespace g2o
