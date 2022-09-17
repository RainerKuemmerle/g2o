#include "py_sparse_block_matrix.h"

#include <g2o/core/sparse_block_matrix.h>

#include "g2opy.h"

namespace g2o {

namespace {
template <class MatrixType = MatrixX>
void templatedSparseBlockMatrix(py::module& m, const std::string& suffix) {
  using CLS = SparseBlockMatrix<MatrixType>;

  py::class_<CLS>(m, ("SparseBlockMatrix" + suffix).c_str())
      .def(py::init<>())
      .def("clear", &CLS::clear, "dealloc"_a = false)
      .def("cols", &CLS::cols)
      .def("rows", &CLS::rows)
      .def("block", (const MatrixType* (CLS::*)(int, int) const) & CLS::block,
           "r"_a, "c"_a)
      // TODO(goki): IMplement the remaining functions of SparseBlockMatrix
      ;
}
}  // namespace

void delcareSparseBlockMatrix(py::module& m) {
  templatedSparseBlockMatrix<MatrixX>(m, "X");
}

}  // namespace g2o
