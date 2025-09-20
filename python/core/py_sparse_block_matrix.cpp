#include "py_sparse_block_matrix.h"

#include <g2o/core/sparse_block_matrix.h>

#include "g2opy.h"

namespace g2o {

namespace {
template <class MatrixType = MatrixX>
void templatedSparseBlockMatrix(py::module& m, const std::string& suffix) {
  using CLS = SparseBlockMatrix<MatrixType>;

  py::classh<CLS>(m, ("SparseBlockMatrix" + suffix).c_str())
      .def(py::init<>())
      .def("clear", &CLS::clear, "dealloc"_a = false)
      .def("cols", &CLS::cols)
      .def("rows", &CLS::rows)
      .def("block",
           static_cast<const MatrixType* (CLS::*)(int, int) const>(&CLS::block),
           "r"_a, "c"_a, pybind11::return_value_policy::reference_internal)
      .def(
          "has_block",
          [](const CLS& sparse_block, int r, int c) {
            auto it = sparse_block.blockCols()[c].find(r);
            return (it != sparse_block.blockCols()[c].end());
          },
          "r"_a, "c"_a)
      .def("rows_of_block", &CLS::rowsOfBlock)
      .def("cols_of_block", &CLS::colsOfBlock)
      .def("row_base_of_block", &CLS::rowBaseOfBlock)
      .def("col_base_of_block", &CLS::colBaseOfBlock)
      .def("non_zeros", &CLS::nonZeros)
      .def("non_zeros_blocks", &CLS::nonZeroBlocks)
      .def("scale", &CLS::scale)
      .def("block_cols",
           static_cast<const std::vector<typename CLS::IntBlockMap>& (
               CLS::*)(void) const>(&CLS::blockCols),
           py::return_value_policy::reference_internal)
      .def("row_block_indices",
           static_cast<const std::vector<int>& (CLS::*)(void) const>(
               &CLS::rowBlockIndices))
      .def("col_block_indices",
           static_cast<const std::vector<int>& (CLS::*)(void) const>(
               &CLS::colBlockIndices))
      .def("write_octave", &CLS::writeOctave, "filename"_a,
           "upperTriangle"_a = false)
      // TODO(goki): Implement the remaining functions of SparseBlockMatrix
      ;
}
}  // namespace

void delcareSparseBlockMatrix(py::module& m) {
  templatedSparseBlockMatrix<MatrixX>(m, "X");
}

}  // namespace g2o
