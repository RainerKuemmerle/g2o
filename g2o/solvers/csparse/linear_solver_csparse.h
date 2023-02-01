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

#ifndef G2O_LINEAR_SOLVERCSPARSE_H
#define G2O_LINEAR_SOLVERCSPARSE_H

#include <iostream>

#include "csparse_wrapper.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/marginal_covariance_cholesky.h"
#include "g2o/stuff/timeutil.h"
#include "g2o_csparse_api.h"

namespace g2o {

/**
 * \brief linear solver which uses CSparse
 */
template <typename MatrixType>
class LinearSolverCSparse : public LinearSolverCCS<MatrixType> {
 public:
  LinearSolverCSparse() : LinearSolverCCS<MatrixType>() {}

  LinearSolverCSparse(LinearSolverCSparse<MatrixType> const&) = delete;
  LinearSolverCSparse& operator=(LinearSolverCSparse<MatrixType> const&) =
      delete;

  virtual ~LinearSolverCSparse() = default;

  bool init() override {
    csparse_.freeSymbolic();
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, number_t* x,
             number_t* b) override {
    prepareSolve(A);

    const number_t t = get_monotonic_time();
    bool ok = csparse_.solve(x, b);
    if (!ok && this->writeDebug()) {
      std::cerr
          << "Cholesky failure, writing debug.txt (Hessian loadable by Octave)"
          << std::endl;
      csparse_.writeSparse("debug.txt");
    }

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeNumericDecomposition = get_monotonic_time() - t;
      globalStats->choleskyNNZ = static_cast<size_t>(csparse_.choleskyNz());
    }

    return ok;
  }

 protected:
  csparse::CSparse csparse_;
  MatrixStructure matrixStructure_;

  void prepareSolve(const SparseBlockMatrix<MatrixType>& A) {
    bool hasSymbolic = csparse_.hasSymbolic();
    fillCSparse(A, hasSymbolic);
    // perform symbolic cholesky once
    if (!hasSymbolic) {
      computeSymbolicDecomposition(A);
      assert(csparse_.hasSymbolic() && "Symbolic cholesky failed");
    }
  }

  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    const number_t t = get_monotonic_time();
    if (!this->blockOrdering()) {
      csparse_.analyze();
    } else {
      A.fillBlockStructure(matrixStructure_);

      // prepare block structure for the CSparse call
      double* structureX = nullptr;
      int structureNz = matrixStructure_.nzMax();
      int structureAllocated = matrixStructure_.n;
      csparse::CSparse::SparseView auxBlock(
          matrixStructure_.n, matrixStructure_.n, structureNz,
          matrixStructure_.Ap, matrixStructure_.Aii, structureX,
          structureAllocated);

      // AMD ordering on the block structure
      VectorXI blockPermutation;
      g2o::csparse::CSparse::amd(auxBlock, blockPermutation);

      // blow up the permutation to the scalar matrix
      VectorXI scalarPermutation;
      this->blockToScalarPermutation(A, blockPermutation, scalarPermutation);
      csparse_.analyze_p(scalarPermutation.data());
    }
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
    }
  }

  void fillCSparse(const SparseBlockMatrix<MatrixType>& A, bool onlyValues) {
    if (!onlyValues) this->initMatrixStructure(A);
    int m = A.rows();
    int n = A.cols();
    assert(m > 0 && n > 0 && "Hessian has 0 rows/cols");

    csparse::CSparse::SparseView ccsA = csparse_.sparseView();

    if (ccsA.columnsAllocated < n) {
      // pre-allocate more space if re-allocating
      ccsA.columnsAllocated = ccsA.columnsAllocated == 0 ? n : 2 * n;
      delete[] ccsA.p;
      ccsA.p = new int[ccsA.columnsAllocated + 1];
    }

    if (!onlyValues) {
      int nzmax = A.nonZeros();
      if (ccsA.nzmax < nzmax) {
        // pre-allocate more space if re-allocating
        ccsA.nzmax = ccsA.nzmax == 0 ? nzmax : 2 * nzmax;
        delete[] ccsA.x;
        delete[] ccsA.i;
        ccsA.i = new int[ccsA.nzmax];
        ccsA.x = new number_t[ccsA.nzmax];
      }
    }
    ccsA.m = m;
    ccsA.n = n;

    if (onlyValues) {
      this->ccsMatrix_->fillCCS(ccsA.x, true);
    } else {
      int nz = this->ccsMatrix_->fillCCS(ccsA.p, ccsA.i, ccsA.x, true);
      (void)nz;
      assert(nz <= ccsA.nzmax);
    }
  }

  /**
   * Implementation of the general parts for computing the inverse blocks of the
   * linear system matrix. Here we call a function to do the underlying
   * computation.
   */
  bool solveBlocks_impl(const SparseBlockMatrix<MatrixType>& A,
                        const std::function<void(MarginalCovarianceCholesky&)>&
                            compute) override {
    prepareSolve(A);
    bool ok = csparse_.factorize();
    if (ok) {
      csparse::CSparse::FactorView factor = csparse_.factor();
      MarginalCovarianceCholesky mcc;
      mcc.setCholeskyFactor(factor.n, factor.p, factor.i, factor.x,
                            factor.pinv);
      compute(mcc);
    } else {
      std::cerr << "inverse fail (numeric decomposition)" << std::endl;
    }
    csparse_.freeFactor();
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->choleskyNNZ = static_cast<size_t>(csparse_.choleskyNz());
    }
    return ok;
  }
};

}  // namespace g2o

#endif
