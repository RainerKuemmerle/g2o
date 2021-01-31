#pragma once

#include <g2o/core/.h>

#include "g2opy.h"

namespace g2o {

void declareSparseOptmizer(py::module& m) {
  py::class_<SparseOptmizer, std::shared_ptr<SparseOptmizer>>(m, "SparseOptmizer")

      ;

  // m.def();
}

}  // end namespace g2o
