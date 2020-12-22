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

#ifndef G2O_LINEAR_SOLVER_H
#define G2O_LINEAR_SOLVER_H

#include <functional>

#include "g2o/core/marginal_covariance_cholesky.h"
#include "sparse_block_matrix.h"
#include "sparse_block_matrix_ccs.h"

namespace g2o {

/**
 * \brief basic solver for Ax = b
 *
 * basic solver for Ax = b which has to reimplemented for different linear algebra libraries.
 * A is assumed to be symmetric (only upper triangular block is stored) and positive-semi-definit.
 */
template <typename MatrixType>
class LinearSolver {
 public:
  LinearSolver() : _writeDebug(true){};
  virtual ~LinearSolver() {}

  /**
   * init for operating on matrices with a different non-zero pattern like before
   */
  virtual bool init() = 0;

  /**
   * Assumes that A is the same matrix for several calls.
   * Among other assumptions, the non-zero pattern does not change!
   * If the matrix changes call init() before.
   * solve system Ax = b, x and b have to allocated beforehand!!
   */
  virtual bool solve(const SparseBlockMatrix<MatrixType>& A, number_t* x, number_t* b) = 0;

  /**
   * Inverts the diagonal blocks of A
   * @returns false if not defined.
   */
  virtual bool solveBlocks(number_t**& blocks, const SparseBlockMatrix<MatrixType>& A) {
    (void)blocks;
    (void)A;
    return false;
  }

  /**
   * Inverts the a block pattern of A in spinv
   * @returns false if not defined.
   */
  virtual bool solvePattern(SparseBlockMatrix<MatrixX>& spinv,
                            const std::vector<std::pair<int, int> >& blockIndices,
                            const SparseBlockMatrix<MatrixType>& A) {
    (void)spinv;
    (void)blockIndices;
    (void)A;
    return false;
  }

  //! write a debug dump of the system matrix if it is not PSD in solve
  bool writeDebug() const { return _writeDebug; }
  void setWriteDebug(bool b) { _writeDebug = b; }

  //! allocate block memory structure
  static void allocateBlocks(const SparseBlockMatrix<MatrixType>& A, number_t**& blocks) {
    blocks = new number_t*[A.rows()];
    number_t** block = blocks;
    for (size_t i = 0; i < A.rowBlockIndices().size(); ++i) {
      int dim = A.rowsOfBlock(i) * A.colsOfBlock(i);
      *block = new number_t[dim];
      block++;
    }
  }

  //! de-allocate the block structure
  static void deallocateBlocks(const SparseBlockMatrix<MatrixType>& A, number_t**& blocks) {
    for (size_t i = 0; i < A.rowBlockIndices().size(); ++i) {
      delete[] blocks[i];
    }
    delete[] blocks;
    blocks = nullptr;
  }

  /**
   * Convert a block permutation matrix to a scalar permutation
   */
  template <typename BlockDerived, typename ScalarDerived>
  static void blockToScalarPermutation(
      const SparseBlockMatrix<MatrixType>& A, const Eigen::MatrixBase<BlockDerived>& p,
      const Eigen::MatrixBase<ScalarDerived>& scalar /* output */) {
    int n = A.cols();
    Eigen::MatrixBase<ScalarDerived>& scalarPermutation =
        const_cast<Eigen::MatrixBase<ScalarDerived>&>(scalar);
    if (scalarPermutation.size() == 0) scalarPermutation.derived().resize(n);
    if (scalarPermutation.size() < n) scalarPermutation.derived().resize(2 * n);
    size_t scalarIdx = 0;
    for (size_t i = 0; i < A.colBlockIndices().size(); ++i) {
      int base = A.colBaseOfBlock(p(i));
      int nCols = A.colsOfBlock(p(i));
      for (int j = 0; j < nCols; ++j) {
        scalarPermutation(scalarIdx++) = base++;
      }
    }
    assert((int)scalarIdx == n);
  }

  protected:
   bool _writeDebug;
};

/**
 * \brief Solver with faster iterating structure for the linear matrix
 */
template <typename MatrixType>
class LinearSolverCCS : public LinearSolver<MatrixType> {
 public:
  LinearSolverCCS() : LinearSolver<MatrixType>(), _ccsMatrix(0), _blockOrdering(true) {}
  ~LinearSolverCCS() { delete _ccsMatrix; }

  virtual bool solveBlocks(number_t**& blocks, const SparseBlockMatrix<MatrixType>& A) {
    auto compute = [&](MarginalCovarianceCholesky& mcc) {
      if (!blocks) LinearSolverCCS<MatrixType>::allocateBlocks(A, blocks);
      mcc.computeCovariance(blocks, A.rowBlockIndices());
    };
    return solveBlocks_impl(A, compute);
  }

  virtual bool solvePattern(SparseBlockMatrix<MatrixX>& spinv,
                            const std::vector<std::pair<int, int> >& blockIndices,
                            const SparseBlockMatrix<MatrixType>& A) {
    auto compute = [&](MarginalCovarianceCholesky& mcc) {
      mcc.computeCovariance(spinv, A.rowBlockIndices(), blockIndices);
    };
    return solveBlocks_impl(A, compute);
  }

  //! do the AMD ordering on the blocks or on the scalar matrix
  bool blockOrdering() const { return _blockOrdering; }
  void setBlockOrdering(bool blockOrdering) { _blockOrdering = blockOrdering; }

 protected:
  SparseBlockMatrixCCS<MatrixType>* _ccsMatrix;
  bool _blockOrdering;

  void initMatrixStructure(const SparseBlockMatrix<MatrixType>& A) {
    delete _ccsMatrix;
    _ccsMatrix = new SparseBlockMatrixCCS<MatrixType>(A.rowBlockIndices(), A.colBlockIndices());
    A.fillSparseBlockMatrixCCS(*_ccsMatrix);
  }

  virtual bool solveBlocks_impl(const SparseBlockMatrix<MatrixType>& A,
                                std::function<void(MarginalCovarianceCholesky&)> compute) = 0;
};

}  // namespace g2o

#endif
