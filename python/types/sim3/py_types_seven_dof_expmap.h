#pragma once

#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2opy.h"
#include "python/core/py_base_binary_edge.h"

namespace g2o {

void declareTypesSevenDofExpmap(py::module& m);

}  // namespace g2o
