#include <g2o/core/.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void declareSparseOptmizer(py::module& m) {
  py::class_<SparseOptmizer, std::shared_ptr<SparseOptmizer>>(m, "SparseOptmizer")

      ;

  m.def();
}

}  // end namespace g2o