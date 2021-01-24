#include <pybind11/pybind11.h>

#include "python/core/py_core.h"
#include "python/types/types.h"

namespace py = pybind11;

namespace g2o {

PYBIND11_MODULE(g2opy, m) {
  declareCore(m);
  declareTypes(m);
}

}  // namespace g2o
