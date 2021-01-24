#include <pybind11/pybind11.h>

namespace py = pybind11;
namespace g2o {

void declareG2OBatchStatistics(py::module& m);

}  // namespace g2o
