#pragma once

#include "g2opy.h"

#include "g2o/core/eigen_types.h"

namespace g2o {

void declareEigenTypes(py::module_& m);

}  // end namespace g2o
