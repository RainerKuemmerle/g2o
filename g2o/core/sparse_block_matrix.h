// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_SPARSE_BLOCK_MATRIX_
#define G2O_SPARSE_BLOCK_MATRIX_

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "g2o/config.h"  // IWYU pragma: keep
#include "g2o/core/eigen_types.h"
#include "g2o/stuff/sparse_helper.h"
#include "matrix_operations.h"
#include "matrix_structure.h"
#include "sparse_block_matrix_ccs.h"

namespace g2o {
/**
 * \brief Sparse matrix which uses blocks
 *
 * Template class that specifies a sparse block matrix.  A block
 * matrix is a sparse matrix made of dense blocks.  These blocks
 * cannot have a random pattern, but follow a (variable) grid
 * structure. This structure is specified by a partition of the rows
 * and the columns of the matrix.  The blocks are represented by the
 * Eigen::Matrix structure, thus they can be statically or dynamically
 * allocated. For efficiency reasons it is convenient to allocate them
 * statically, when possible. A static block matrix has all blocks of
 * the same size, and the size of the block is specified by the
 * template argument.  If this is not the case, and you have different
 * block sizes than you have to use a dynamic-block matrix (default
 * template argument).
 */
template <class MatrixType = MatrixX>
class SparseBlockMatrix {
 public:
  //! this is the type of the elementary block, it is an Eigen::Matrix.
  using SparseMatrixBlock = MatrixType;

  //! columns of the matrix
  [[nodiscard]] int cols() const {
    return !colBlockIndices_.empty() ? colBlockIndices_.back() : 0;
  }
  //! rows of the matrix
  [[nodiscard]] int rows() const {
    return !rowBlockIndices_.empty() ? rowBlockIndices_.back() : 0;
  }

  using IntBlockMap = std::map<int, SparseMatrixBlock*>;

  /**
   * constructs a sparse block matrix having a specific layout
   * @param rbi: array of int containing the row layout of the blocks.
   * the component i of the array should contain the index of the first row of
   * the block i+1.
   * @param cbi: array of int containing the column layout of the blocks.
   *  the component i of the array should contain the index of the first col of
   * the block i+1.
   * @param rb: number of row blocks
   * @param cb: number of col blocks
   * @param hasStorage: set it to true if the matrix "owns" the blocks, thus it
   * deletes it on destruction. if false the matrix is only a "view" over an
   * existing structure.
   */
  SparseBlockMatrix(const int* rbi, const int* cbi, int rb, int cb,
                    bool hasStorage = true);

  SparseBlockMatrix();

  ~SparseBlockMatrix();

  //! this zeroes all the blocks. If dealloc=true the blocks are removed from
  //! memory
  void clear(bool dealloc = false);

  //! returns the block at location r,c. if alloc=true he block is created if it
  //! does not exist
  SparseMatrixBlock* block(int r, int c, bool alloc = false);
  //! returns the block at location r,c
  const SparseMatrixBlock* block(int r, int c) const;

  //! how many rows does the block at block-row r has?
  [[nodiscard]] int rowsOfBlock(int r) const {
    return r ? rowBlockIndices_[r] - rowBlockIndices_[r - 1]
             : rowBlockIndices_[0];
  }

  //! how many cols does the block at block-col c has?
  [[nodiscard]] int colsOfBlock(int c) const {
    return c ? colBlockIndices_[c] - colBlockIndices_[c - 1]
             : colBlockIndices_[0];
  }

  //! where does the row at block-row r starts?
  [[nodiscard]] int rowBaseOfBlock(int r) const {
    return r ? rowBlockIndices_[r - 1] : 0;
  }

  //! where does the col at block-col r starts?
  [[nodiscard]] int colBaseOfBlock(int c) const {
    return c ? colBlockIndices_[c - 1] : 0;
  }

  //! number of non-zero elements
  [[nodiscard]] size_t nonZeros() const;
  //! number of allocated blocks
  [[nodiscard]] size_t nonZeroBlocks() const;

  //! deep copy of a sparse-block-matrix;
  SparseBlockMatrix* clone() const;

  /**
   * returns a view or a copy of the block matrix
   * @param rmin: starting block row
   * @param rmax: ending  block row
   * @param cmin: starting block col
   * @param cmax: ending  block col
   * @param alloc: if true it makes a deep copy, if false it creates a view.
   */
  SparseBlockMatrix* slice(int rmin, int rmax, int cmin, int cmax,
                           bool alloc = true) const;

  //! transposes a block matrix, The transposed type should match the argument
  //! false on failure
  template <class MatrixTransposedType>
  bool transpose(SparseBlockMatrix<MatrixTransposedType>& dest) const;

  template <class MatrixTransposedType>
  std::unique_ptr<SparseBlockMatrix<MatrixTransposedType>> transposed() const;

  //! adds the current matrix to the destination
  bool add(SparseBlockMatrix<MatrixType>& dest) const;
  std::unique_ptr<SparseBlockMatrix<MatrixType>> added() const;

  //! dest = (*this) *  M
  template <class MatrixResultType, class MatrixFactorType>
  bool multiply(SparseBlockMatrix<MatrixResultType>*& dest,
                const SparseBlockMatrix<MatrixFactorType>* M) const;

  //! dest = (*this) *  src
  void multiply(double*& dest, const double* src) const;

  /**
   * compute dest = (*this) *  src
   * However, assuming that this is a symmetric matrix where only the upper
   * triangle is stored
   */
  void multiplySymmetricUpperTriangle(double*& dest, const double* src) const;

  //! dest = M * (*this)
  void rightMultiply(double*& dest, const double* src) const;

  //! *this *= a
  void scale(double a);

  /**
   * writes in dest a block permutaton specified by pinv.
   * @param pinv: array such that new_block[i] = old_block[pinv[i]]
   */
  bool symmPermutation(SparseBlockMatrix<MatrixType>*& dest, const int* pinv,
                       bool onlyUpper = false) const;

  /**
   * fill the CCS arrays of a matrix, arrays have to be allocated beforehand
   */
  int fillCCS(int* Cp, int* Ci, double* Cx, bool upperTriangle = false) const;

  /**
   * fill the CCS arrays of a matrix, arrays have to be allocated beforehand.
   * This function only writes the values and assumes that column and row
   * structures have already been written.
   */
  int fillCCS(double* Cx, bool upperTriangle = false) const;

  //! exports the non zero blocks in the structure matrix ms
  void fillBlockStructure(MatrixStructure& ms) const;

  //! exports the non zero blocks into a column compressed structure
  void fillBlockStructure(int* Cp, int* Ci) const;

  //! the block matrices per block-column
  const std::vector<IntBlockMap>& blockCols() const { return blockCols_; }
  std::vector<IntBlockMap>& blockCols() { return blockCols_; }

  //! indices of the row blocks
  [[nodiscard]] const std::vector<int>& rowBlockIndices() const {
    return rowBlockIndices_;
  }
  std::vector<int>& rowBlockIndices() { return rowBlockIndices_; }

  //! indices of the column blocks
  [[nodiscard]] const std::vector<int>& colBlockIndices() const {
    return colBlockIndices_;
  }
  std::vector<int>& colBlockIndices() { return colBlockIndices_; }

  /**
   * write the content of this matrix to a stream loadable by Octave
   * @param upperTriangle does this matrix store only the upper triangular
   * blocks
   */
  bool writeOctave(const char* filename, bool upperTriangle = true) const;

  /**
   * copy into CCS structure
   * @return number of processed blocks, -1 on error
   */
  int fillSparseBlockMatrixCCS(
      SparseBlockMatrixCCS<MatrixType>& blockCCS) const;

  /**
   * copy as transposed into a CCS structure
   * @return number of processed blocks, -1 on error
   */
  int fillSparseBlockMatrixCCSTransposed(
      SparseBlockMatrixCCS<MatrixType>& blockCCS) const;

  /**
   * take over the memory and matrix pattern from a hash matrix.
   * The structure of the hash matrix will be cleared.
   */
  void takePatternFromHash(SparseBlockMatrixHashMap<MatrixType>& hashMatrix);

 protected:
  std::vector<int> rowBlockIndices_;  ///< vector of the indices of the blocks
                                      ///< along the rows.
  std::vector<int>
      colBlockIndices_;  ///< vector of the indices of the blocks along the cols
  //! array of maps of blocks. The index of the array represent a block column
  //! of the matrix and the block column is stored as a map row_block ->
  //! matrix_block_ptr.
  std::vector<IntBlockMap> blockCols_;
  bool hasStorage_{true};

 private:
  template <class MatrixTransposedType>
  void transpose_internal(SparseBlockMatrix<MatrixTransposedType>& dest) const;

  void add_internal(SparseBlockMatrix<MatrixType>& dest) const;
};

template <class MatrixType>
std::ostream& operator<<(std::ostream&, const SparseBlockMatrix<MatrixType>& m);

template <class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix(const int* rbi, const int* cbi,
                                                 int rb, int cb,
                                                 bool hasStorage)
    : rowBlockIndices_(rbi, rbi + rb),
      colBlockIndices_(cbi, cbi + cb),
      blockCols_(cb),
      hasStorage_(hasStorage) {}

template <class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix() : blockCols_(0) {}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::clear(bool dealloc) {
#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) if (blockCols_.size() > 100)
#endif
  for (int i = 0; i < static_cast<int>(blockCols_.size()); ++i) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = blockCols_[i].begin();
         it != blockCols_[i].end(); ++it) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b = it->second;
      if (hasStorage_ && dealloc)
        delete b;
      else
        b->setZero();
    }
    if (hasStorage_ && dealloc) blockCols_[i].clear();
  }
}

template <class MatrixType>
SparseBlockMatrix<MatrixType>::~SparseBlockMatrix() {
  if (hasStorage_) clear(true);
}

template <class MatrixType>
typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock*
SparseBlockMatrix<MatrixType>::block(int r, int c, bool alloc) {
  auto it = blockCols_[c].find(r);
  typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* _block = nullptr;
  if (it == blockCols_[c].end()) {
    if (!hasStorage_ && !alloc) return nullptr;
    int rb = rowsOfBlock(r);
    int cb = colsOfBlock(c);
    _block =
        new typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock(rb, cb);
    _block->setZero();
    std::pair<typename SparseBlockMatrix<MatrixType>::IntBlockMap::iterator,
              bool>
        result = blockCols_[c].insert(std::make_pair(r, _block));
    (void)result;
    assert(result.second);

  } else {
    _block = it->second;
  }
  return _block;
}

template <class MatrixType>
const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock*
SparseBlockMatrix<MatrixType>::block(int r, int c) const {
  auto it = blockCols_[c].find(r);
  if (it == blockCols_[c].end()) return nullptr;
  return it->second;
}

template <class MatrixType>
SparseBlockMatrix<MatrixType>* SparseBlockMatrix<MatrixType>::clone() const {
  auto* ret =
      new SparseBlockMatrix(rowBlockIndices_.data(), colBlockIndices_.data(),
                            rowBlockIndices_.size(), colBlockIndices_.size());
  for (size_t i = 0; i < blockCols_.size(); ++i) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = blockCols_[i].begin();
         it != blockCols_[i].end(); ++it) {
      auto* b = new typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock(
          *it->second);
      ret->blockCols_[i].insert(std::make_pair(it->first, b));
    }
  }
  ret->hasStorage_ = true;
  return ret;
}

template <class MatrixType>
template <class MatrixTransposedType>
void SparseBlockMatrix<MatrixType>::transpose_internal(
    SparseBlockMatrix<MatrixTransposedType>& dest) const {
  for (size_t i = 0; i < blockCols_.size(); ++i) {
    for (const auto& block : blockCols_[i]) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s =
          block.second;
      typename SparseBlockMatrix<MatrixTransposedType>::SparseMatrixBlock* d =
          dest.block(i, block.first, true);
      *d = s->transpose();
    }
  }
}

template <class MatrixType>
template <class MatrixTransposedType>
bool SparseBlockMatrix<MatrixType>::transpose(
    SparseBlockMatrix<MatrixTransposedType>& dest) const {
  if (!dest.hasStorage_) return false;
  if (rowBlockIndices_.size() != dest.colBlockIndices_.size()) return false;
  if (colBlockIndices_.size() != dest.rowBlockIndices_.size()) return false;
  for (size_t i = 0; i < rowBlockIndices_.size(); ++i) {
    if (rowBlockIndices_[i] != dest.colBlockIndices_[i]) return false;
  }
  for (size_t i = 0; i < colBlockIndices_.size(); ++i) {
    if (colBlockIndices_[i] != dest.rowBlockIndices_[i]) return false;
  }

  transpose_internal(dest);
  return true;
}

template <class MatrixType>
template <class MatrixTransposedType>
std::unique_ptr<SparseBlockMatrix<MatrixTransposedType>>
SparseBlockMatrix<MatrixType>::transposed() const {
  auto dest = std::make_unique<SparseBlockMatrix<MatrixTransposedType>>(
      colBlockIndices_.data(), rowBlockIndices_.data(), colBlockIndices_.size(),
      rowBlockIndices_.size());
  transpose_internal(*dest);
  return dest;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::add_internal(
    SparseBlockMatrix<MatrixType>& dest) const {
  for (size_t i = 0; i < blockCols_.size(); ++i) {
    for (auto it = blockCols_[i].begin(); it != blockCols_[i].end(); ++it) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s = it->second;
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* d =
          dest.block(it->first, i, true);
      (*d) += *s;
    }
  }
}

template <class MatrixType>
bool SparseBlockMatrix<MatrixType>::add(
    SparseBlockMatrix<MatrixType>& dest) const {
  if (!dest.hasStorage_) return false;
  if (rowBlockIndices_.size() != dest.rowBlockIndices_.size()) return false;
  if (colBlockIndices_.size() != dest.colBlockIndices_.size()) return false;
  for (size_t i = 0; i < rowBlockIndices_.size(); ++i) {
    if (rowBlockIndices_[i] != dest.rowBlockIndices_[i]) return false;
  }
  for (size_t i = 0; i < colBlockIndices_.size(); ++i) {
    if (colBlockIndices_[i] != dest.colBlockIndices_[i]) return false;
  }

  add_internal(dest);
  return true;
}

template <class MatrixType>
std::unique_ptr<SparseBlockMatrix<MatrixType>>
SparseBlockMatrix<MatrixType>::added() const {
  auto a = std::make_unique<SparseBlockMatrix>(
      rowBlockIndices_.data(), colBlockIndices_.data(), rowBlockIndices_.size(),
      colBlockIndices_.size());
  add_internal(*a);
  return a;
}

template <class MatrixType>
template <class MatrixResultType, class MatrixFactorType>
bool SparseBlockMatrix<MatrixType>::multiply(
    SparseBlockMatrix<MatrixResultType>*& dest,
    const SparseBlockMatrix<MatrixFactorType>* M) const {
  // sanity check
  if (colBlockIndices_.size() != M->rowBlockIndices_.size()) return false;
  for (size_t i = 0; i < colBlockIndices_.size(); ++i) {
    if (colBlockIndices_[i] != M->rowBlockIndices_[i]) return false;
  }
  if (!dest) {
    dest = new SparseBlockMatrix<MatrixResultType>(
        rowBlockIndices_.data(), M->colBlockIndices_.data(),
        rowBlockIndices_.size(), M->colBlockIndices_.size());
  }
  if (!dest->hasStorage_) return false;
  for (size_t i = 0; i < M->blockCols_.size(); ++i) {
    for (const auto& block : M->blockCols_[i]) {
      // look for a non-zero block in a row of column it
      int colM = i;
      const auto* b = block.second;
      auto rbt = blockCols_[block.first].begin();
      while (rbt != blockCols_[block.first].end()) {
        // int colA=block.first;
        int rowA = rbt->first;
        typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
            rbt->second;
        typename SparseBlockMatrix<MatrixResultType>::SparseMatrixBlock* c =
            dest->block(rowA, colM, true);
        assert(c->rows() == a->rows());
        assert(c->cols() == b->cols());
        ++rbt;
        (*c) += (*a) * (*b);
      }
    }
  }
  return true;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::multiply(double*& dest,
                                             const double* src) const {
  if (!dest) {
    dest = new double[rowBlockIndices_[rowBlockIndices_.size() - 1]];
    memset(dest, 0,
           rowBlockIndices_[rowBlockIndices_.size() - 1] * sizeof(double));
  }

  // map the memory by Eigen
  Eigen::Map<VectorX> destVec(dest, rows());
  const Eigen::Map<const VectorX> srcVec(src, cols());

  for (size_t i = 0; i < blockCols_.size(); ++i) {
    int srcOffset = i ? colBlockIndices_[i - 1] : 0;

    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = blockCols_[i].begin();
         it != blockCols_[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second;
      int destOffset = it->first ? rowBlockIndices_[it->first - 1] : 0;
      // destVec += *a * srcVec (according to the sub-vector parts)
      internal::template axpy<
          typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
          *a, srcVec, srcOffset, destVec, destOffset);
    }
  }
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::multiplySymmetricUpperTriangle(
    double*& dest, const double* src) const {
  if (!dest) {
    dest = new double[rowBlockIndices_[rowBlockIndices_.size() - 1]];
    memset(dest, 0,
           rowBlockIndices_[rowBlockIndices_.size() - 1] * sizeof(double));
  }

  // map the memory by Eigen
  Eigen::Map<VectorX> destVec(dest, rows());
  const Eigen::Map<const VectorX> srcVec(src, cols());

  for (size_t i = 0; i < blockCols_.size(); ++i) {
    int srcOffset = colBaseOfBlock(i);
    for (auto it = blockCols_[i].begin(); it != blockCols_[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second;
      int destOffset = rowBaseOfBlock(it->first);
      if (destOffset > srcOffset)  // only upper triangle
        break;
      // destVec += *a * srcVec (according to the sub-vector parts)
      internal::template axpy<
          typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
          *a, srcVec, srcOffset, destVec, destOffset);
      if (destOffset < srcOffset)
        internal::template atxpy<
            typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
            *a, srcVec, destOffset, destVec, srcOffset);
    }
  }
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::rightMultiply(double*& dest,
                                                  const double* src) const {
  int destSize = cols();

  if (!dest) {
    dest = new double[destSize];
    memset(dest, 0, destSize * sizeof(double));
  }

  // map the memory by Eigen
  Eigen::Map<VectorX> destVec(dest, destSize);
  Eigen::Map<const VectorX> srcVec(src, rows());

#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) schedule(dynamic, 10)
#endif
  for (int i = 0; i < static_cast<int>(blockCols_.size()); ++i) {
    int destOffset = colBaseOfBlock(i);
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = blockCols_[i].begin();
         it != blockCols_[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second;
      int srcOffset = rowBaseOfBlock(it->first);
      // destVec += *a.transpose() * srcVec (according to the sub-vector parts)
      internal::template atxpy<
          typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
          *a, srcVec, srcOffset, destVec, destOffset);
    }
  }
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::scale(double a_) {
  for (size_t i = 0; i < blockCols_.size(); ++i) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = blockCols_[i].begin();
         it != blockCols_[i].end(); ++it) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a = it->second;
      *a *= a_;
    }
  }
}

template <class MatrixType>
SparseBlockMatrix<MatrixType>* SparseBlockMatrix<MatrixType>::slice(
    int rmin, int rmax, int cmin, int cmax, bool alloc) const {
  const int m = rmax - rmin;
  const int n = cmax - cmin;
  std::vector<int> rowIdx(m);
  rowIdx[0] = rowsOfBlock(rmin);
  for (int i = 1; i < m; ++i) {
    rowIdx[i] = rowIdx[i - 1] + rowsOfBlock(rmin + i);
  }

  std::vector<int> colIdx(n);
  colIdx[0] = colsOfBlock(cmin);
  for (int i = 1; i < n; ++i) {
    colIdx[i] = colIdx[i - 1] + colsOfBlock(cmin + i);
  }
  auto* s = new SparseBlockMatrix(rowIdx, colIdx, m, n, true);
  for (int i = 0; i < n; ++i) {
    int mc = cmin + i;
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = blockCols_[mc].begin();
         it != blockCols_[mc].end(); ++it) {
      if (it->first >= rmin && it->first < rmax) {
        typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
            alloc ? new
                typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock(
                    *(it->second))
                  : it->second;
        s->blockCols_[i].insert(std::make_pair(it->first - rmin, b));
      }
    }
  }
  s->hasStorage_ = alloc;
  return s;
}

template <class MatrixType>
size_t SparseBlockMatrix<MatrixType>::nonZeroBlocks() const {
  size_t count = 0;
  for (size_t i = 0; i < blockCols_.size(); ++i) count += blockCols_[i].size();
  return count;
}

template <class MatrixType>
size_t SparseBlockMatrix<MatrixType>::nonZeros() const {
  if (MatrixType::SizeAtCompileTime != Eigen::Dynamic) {
    size_t nnz = nonZeroBlocks() * MatrixType::SizeAtCompileTime;
    return nnz;
  }
  size_t count = 0;
  for (size_t i = 0; i < blockCols_.size(); ++i) {
    for (auto it = blockCols_[i].begin(); it != blockCols_[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second;
      count += a->cols() * a->rows();
    }
  }
  return count;
}

template <class MatrixType>
std::ostream& operator<<(std::ostream& os,
                         const SparseBlockMatrix<MatrixType>& m) {
  os << "RBI: " << m.rowBlockIndices().size();
  for (size_t i = 0; i < m.rowBlockIndices().size(); ++i)
    os << " " << m.rowBlockIndices()[i];
  os << '\n';
  os << "CBI: " << m.colBlockIndices().size();
  for (size_t i = 0; i < m.colBlockIndices().size(); ++i)
    os << " " << m.colBlockIndices()[i];
  os << '\n';

  for (size_t i = 0; i < m.blockCols().size(); ++i) {
    for (auto it = m.blockCols()[i].begin(); it != m.blockCols()[i].end();
         ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
          it->second;
      os << "BLOCK: " << it->first << " " << i << '\n';
      os << *b << '\n';
    }
  }
  return os;
}

template <class MatrixType>
bool SparseBlockMatrix<MatrixType>::symmPermutation(
    SparseBlockMatrix<MatrixType>*& dest, const int* pinv,
    bool onlyUpper) const {
  // compute the permuted version of the new row/column layout
  size_t n = rowBlockIndices_.size();
  // computed the block sizes
  std::vector<int> blockSizes(rowBlockIndices_.size());
  blockSizes[0] = rowBlockIndices_[0];
  for (size_t i = 1; i < n; ++i) {
    blockSizes[i] = rowBlockIndices_[i] - rowBlockIndices_[i - 1];
  }
  // permute them
  std::vector<int> pBlockIndices(rowBlockIndices_.size());
  for (size_t i = 0; i < n; ++i) {
    pBlockIndices[pinv[i]] = blockSizes[i];
  }
  for (size_t i = 1; i < n; ++i) {
    pBlockIndices[i] += pBlockIndices[i - 1];
  }
  // allocate C, or check the structure;
  if (!dest) {
    dest =
        new SparseBlockMatrix(pBlockIndices.data(), pBlockIndices.data(), n, n);
  } else {
    if (dest->rowBlockIndices_.size() != n) return false;
    if (dest->colBlockIndices_.size() != n) return false;
    for (size_t i = 0; i < n; ++i) {
      if (dest->rowBlockIndices_[i] != pBlockIndices[i]) return false;
      if (dest->colBlockIndices_[i] != pBlockIndices[i]) return false;
    }
    dest->clear();
  }
  // now ready to permute the columns
  for (size_t i = 0; i < n; ++i) {
    int pi = pinv[i];
    for (const auto& block : blockCols_[i]) {
      int pj = pinv[block.first];

      const auto* s = block.second;
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b = nullptr;
      if (!onlyUpper || pj <= pi) {
        b = dest->block(pj, pi, true);
        assert(b->cols() == s->cols());
        assert(b->rows() == s->rows());
        *b = *s;
      } else {
        b = dest->block(pi, pj, true);
        assert(b);
        assert(b->rows() == s->cols());
        assert(b->cols() == s->rows());
        *b = s->transpose();
      }
    }
    //  within each row,
  }
  return true;
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::fillCCS(double* Cx,
                                           bool upperTriangle) const {
  assert(Cx && "Target destination is NULL");
  double* CxStart = Cx;
  for (size_t i = 0; i < blockCols_.size(); ++i) {
    int cstart = i ? colBlockIndices_[i - 1] : 0;
    int csize = colsOfBlock(i);
    for (int c = 0; c < csize; ++c) {
      for (auto it = blockCols_[i].begin(); it != blockCols_[i].end(); ++it) {
        const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
            it->second;
        int rstart = it->first ? rowBlockIndices_[it->first - 1] : 0;

        int elemsToCopy = b->rows();
        if (upperTriangle && rstart == cstart) elemsToCopy = c + 1;
        memcpy(Cx, b->data() + (c * b->rows()), elemsToCopy * sizeof(double));
        Cx += elemsToCopy;
      }
    }
  }
  return Cx - CxStart;
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::fillCCS(int* Cp, int* Ci, double* Cx,
                                           bool upperTriangle) const {
  assert(Cp && Ci && Cx && "Target destination is NULL");
  int nz = 0;
  for (size_t i = 0; i < blockCols_.size(); ++i) {
    int cstart = i ? colBlockIndices_[i - 1] : 0;
    int csize = colsOfBlock(i);
    for (int c = 0; c < csize; ++c) {
      *Cp = nz;
      for (auto it = blockCols_[i].begin(); it != blockCols_[i].end(); ++it) {
        const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
            it->second;
        int rstart = it->first ? rowBlockIndices_[it->first - 1] : 0;

        int elemsToCopy = b->rows();
        if (upperTriangle && rstart == cstart) elemsToCopy = c + 1;
        for (int r = 0; r < elemsToCopy; ++r) {
          *Cx++ = (*b)(r, c);
          *Ci++ = rstart++;
          ++nz;
        }
      }
      ++Cp;
    }
  }
  *Cp = nz;
  return nz;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::fillBlockStructure(
    MatrixStructure& ms) const {
  ms.alloc(colBlockIndices_.size(), nonZeroBlocks());
  ms.m = rowBlockIndices_.size();
  fillBlockStructure(ms.Ap, ms.Aii);
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::fillBlockStructure(int* Cp, int* Ci) const {
  int nz = 0;
  for (int c = 0; c < static_cast<int>(blockCols_.size()); ++c) {
    *Cp = nz;
    for (auto it = blockCols_[c].begin(); it != blockCols_[c].end(); ++it) {
      const int& r = it->first;
      if (r <= c) {
        *Ci++ = r;
        ++nz;
      }
    }
    Cp++;
  }
  *Cp = nz;
  assert(nz <= static_cast<int>(nonZeroBlocks()));
}

template <class MatrixType>
bool SparseBlockMatrix<MatrixType>::writeOctave(const char* filename,
                                                bool upperTriangle) const {
  std::string name = filename;
  std::string::size_type lastDot = name.find_last_of('.');
  if (lastDot != std::string::npos) name.resize(lastDot);

  std::vector<TripletEntry> entries;
  for (size_t i = 0; i < blockCols_.size(); ++i) {
    const int& c = i;
    for (auto it = blockCols_[i].begin(); it != blockCols_[i].end(); ++it) {
      const int& r = it->first;
      const MatrixType& m = *(it->second);
      for (int cc = 0; cc < m.cols(); ++cc)
        for (int rr = 0; rr < m.rows(); ++rr) {
          int aux_r = rowBaseOfBlock(r) + rr;
          int aux_c = colBaseOfBlock(c) + cc;
          entries.push_back(TripletEntry(aux_r, aux_c, m(rr, cc)));
          if (upperTriangle && r != c) {
            entries.push_back(TripletEntry(aux_c, aux_r, m(rr, cc)));
          }
        }
    }
  }

  int nz = entries.size();
  std::sort(entries.begin(), entries.end(), TripletColSort());

  std::ofstream fout(filename);
  fout << "# name: " << name << '\n';
  fout << "# type: sparse matrix\n";
  fout << "# nnz: " << nz << '\n';
  fout << "# rows: " << rows() << '\n';
  fout << "# columns: " << cols() << '\n';
  fout << std::setprecision(9) << std::fixed << '\n';

  for (auto entry : entries) {
    fout << entry.r + 1 << " " << entry.c + 1 << " " << entry.x << '\n';
  }
  return fout.good();
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::fillSparseBlockMatrixCCS(
    SparseBlockMatrixCCS<MatrixType>& blockCCS) const {
  blockCCS.blockCols().resize(blockCols().size());
  int numblocks = 0;
  for (size_t i = 0; i < blockCols().size(); ++i) {
    const IntBlockMap& row = blockCols()[i];
    typename SparseBlockMatrixCCS<MatrixType>::SparseColumn& dest =
        blockCCS.blockCols()[i];
    dest.clear();
    dest.reserve(row.size());
    for (auto it = row.begin(); it != row.end(); ++it) {
      dest.push_back(typename SparseBlockMatrixCCS<MatrixType>::RowBlock(
          it->first, it->second));
      ++numblocks;
    }
  }
  return numblocks;
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::fillSparseBlockMatrixCCSTransposed(
    SparseBlockMatrixCCS<MatrixType>& blockCCS) const {
  blockCCS.blockCols().clear();
  blockCCS.blockCols().resize(rowBlockIndices_.size());
  int numblocks = 0;
  for (size_t i = 0; i < blockCols().size(); ++i) {
    const IntBlockMap& row = blockCols()[i];
    for (auto it = row.begin(); it != row.end(); ++it) {
      typename SparseBlockMatrixCCS<MatrixType>::SparseColumn& dest =
          blockCCS.blockCols()[it->first];
      dest.push_back(
          typename SparseBlockMatrixCCS<MatrixType>::RowBlock(i, it->second));
      ++numblocks;
    }
  }
  return numblocks;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::takePatternFromHash(
    SparseBlockMatrixHashMap<MatrixType>& hashMatrix) {
  // sort the sparse columns and add them to the map structures by
  // exploiting that we are inserting a sorted structure
  using SparseColumnPair = std::pair<int, MatrixType*>;
  using HashSparseColumn =
      typename SparseBlockMatrixHashMap<MatrixType>::SparseColumn;
  for (size_t i = 0; i < hashMatrix.blockCols().size(); ++i) {
    // prepare a temporary vector for sorting
    HashSparseColumn& column = hashMatrix.blockCols()[i];
    if (column.size() == 0) continue;
    std::vector<SparseColumnPair> sparseRowSorted;  // temporary structure
    sparseRowSorted.reserve(column.size());
    for (typename HashSparseColumn::const_iterator it = column.begin();
         it != column.end(); ++it)
      sparseRowSorted.push_back(*it);
    std::sort(
        sparseRowSorted.begin(), sparseRowSorted.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    // try to free some memory early
    HashSparseColumn aux;
    std::swap(aux, column);
    // now insert sorted vector to the std::map structure
    IntBlockMap& destColumnMap = blockCols()[i];
    destColumnMap.insert(sparseRowSorted[0]);
    for (size_t j = 1; j < sparseRowSorted.size(); ++j) {
      auto hint = destColumnMap.end();
      --hint;  // cppreference says the element goes after the hint (until
               // C++11)
      destColumnMap.insert(hint, sparseRowSorted[j]);
    }
  }
}

using SparseBlockMatrixX = SparseBlockMatrix<MatrixX>;

}  // namespace g2o

#endif
