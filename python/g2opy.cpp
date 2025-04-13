#include "detail/registry.h"
#include "python/core/py_core.h"
#include "python/types/py_types.h"

#ifdef HAVE_G2O_SIMULATOR
#include "python/simulator/py_simulator.h"
#endif

namespace g2o {

PYBIND11_MODULE(g2opy, m) {
  declareCore(m);

  g2o::detail::Registry registry(m);
  declareTypes(registry);

#ifdef HAVE_G2O_SIMULATOR
  declareSimulator(m);
#endif
}

}  // namespace g2o
