#include "g2opy.h"

#include "python/core/py_core.h"
#include "python/types/py_types.h"

namespace g2o {

PYBIND11_MODULE(g2opy, m) {
  declareCore(m);
  declareTypes(m);
}

}  // namespace g2o
