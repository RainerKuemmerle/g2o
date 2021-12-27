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

#include "csparse_helper.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/marginal_covariance_cholesky.h"
#include "g2o/stuff/timeutil.h"
#include "g2o_csparse_api.h"

namespace g2o {

/**
 * \brief Our C++ version of the csparse struct
 */
struct G2O_SOLVER_CSPARSE_API CSparseExt : public cs {
  CSparseExt() {
    nzmax = 0;
    m = 0;
    n = 0;
    p = nullptr;
    i = nullptr;
    x = nullptr;
    nz = 0;
    columnsAllocated = 0;
  }
  CSparseExt(CSparseExt const&) = delete;
  CSparseExt& operator=(CSparseExt const&) = delete;
  ~CSparseExt() {
    delete[] p;
    delete[] i;
    delete[] x;
  }
  int columnsAllocated;
};

/**
 * \brief linear solver which uses CSparse
 */
template <typename MatrixType>
class LinearSolverCSparse : public LinearSolverCCS<MatrixType> {
 public:
  LinearSolverCSparse()
      : LinearSolverCCS<MatrixType>(), symbolicDecomposition_(nullptr) {}

  LinearSolverCSparse(LinearSolverCSparse<MatrixType> const&) = delete;
  LinearSolverCSparse& operator=(LinearSolverCSparse<MatrixType> const&) =
      delete;

  ~LinearSolverCSparse() override {
    if (symbolicDecomposition_) {
      cs_sfree(symbolicDecomposition_);
      symbolicDecomposition_ = nullptr;
    }
    delete[] csWorkspace_;
    csWorkspace_ = nullptr;
    delete[] csIntWorkspace_;
    csIntWorkspace_ = nullptr;
  }

  bool init() override {
    if (symbolicDecomposition_) {
      cs_sfree(symbolicDecomposition_);
      symbolicDecomposition_ = nullptr;
    }
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, number_t* x,
             number_t* b) override {
    prepareSolve(A);

    number_t t = get_monotonic_time();
    // _x = _b for calling csparse
    if (x != b) memcpy(x, b, ccsA_.n * sizeof(number_t));
    int ok = csparse_extension::cs_cholsolsymb(
        &ccsA_, x, symbolicDecomposition_, csWorkspace_, csIntWorkspace_);
    if (!ok && this->writeDebug()) {
      std::cerr
          << "Cholesky failure, writing debug.txt (Hessian loadable by Octave)"
          << std::endl;
      csparse_extension::writeCs2Octave("debug.txt", &ccsA_, true);
    }

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeNumericDecomposition = get_monotonic_time() - t;
      globalStats->choleskyNNZ =
          static_cast<size_t>(symbolicDecomposition_->lnz);
    }

    return ok != 0;
  }

 protected:
  css* symbolicDecomposition_;
  int csWorkspaceSize_ = -1;
  number_t* csWorkspace_ = nullptr;
  int* csIntWorkspace_ = nullptr;
  CSparseExt ccsA_;
  MatrixStructure matrixStructure_;
  VectorXI scalarPermutation_;

  void prepareSolve(const SparseBlockMatrix<MatrixType>& A) {
    fillCSparse(A, symbolicDecomposition_ != nullptr);
    // perform symbolic cholesky once
    if (symbolicDecomposition_ == nullptr) {
      computeSymbolicDecomposition(A);
      assert(_symbolicDecomposition && "Symbolic cholesky failed");
    }
    // re-allocate the temporary workspace for cholesky
    if (csWorkspaceSize_ < ccsA_.n) {
      csWorkspaceSize_ = 2 * ccsA_.n;
      delete[] csWorkspace_;
      csWorkspace_ = new number_t[csWorkspaceSize_];
      delete[] csIntWorkspace_;
      csIntWorkspace_ = new int[2L * csWorkspaceSize_];
    }
  }

  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    number_t t = get_monotonic_time();
    if (!this->blockOrdering()) {
      symbolicDecomposition_ = cs_schol(1, &ccsA_);
    } else {
      A.fillBlockStructure(matrixStructure_);

      // prepare block structure for the CSparse call
      cs auxBlock;
      auxBlock.nzmax = matrixStructure_.nzMax();
      auxBlock.m = auxBlock.n = matrixStructure_.n;
      auxBlock.p = matrixStructure_.Ap;
      auxBlock.i = matrixStructure_.Aii;
      auxBlock.x = nullptr;  // no values
      auxBlock.nz = -1;      // CCS format

      // AMD ordering on the block structure
      const int& n = ccsA_.n;
      int* P = cs_amd(1, &auxBlock);

      // blow up the permutation to the scalar matrix
      VectorXI::MapType blockPermutation(P, matrixStructure_.n);
      this->blockToScalarPermutation(A, blockPermutation, scalarPermutation_);
      cs_free(P);  // clean the memory

      // apply the scalar permutation to finish symbolic decomposition
      symbolicDecomposition_ =
          static_cast<css*>(cs_calloc(1, sizeof(css))); /* allocate result S */
      symbolicDecomposition_->pinv = cs_pinv(scalarPermutation_.data(), n);
      cs* C = cs_symperm(&ccsA_, symbolicDecomposition_->pinv, 0);
      symbolicDecomposition_->parent = cs_etree(C, 0);
      int* post = cs_post(symbolicDecomposition_->parent, n);
      int* c = cs_counts(C, symbolicDecomposition_->parent, post, 0);
      cs_free(post);
      cs_spfree(C);
      symbolicDecomposition_->cp =
          static_cast<int*>(cs_malloc(n + 1, sizeof(int)));
      symbolicDecomposition_->unz = symbolicDecomposition_->lnz =
          cs_cumsum(symbolicDecomposition_->cp, c, n);
      cs_free(c);
      if (symbolicDecomposition_->lnz < 0) {
        cs_sfree(symbolicDecomposition_);
        symbolicDecomposition_ = nullptr;
      }
    }
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
    }

    /* std::cerr << "# Number of nonzeros in L: " <<
     * (int)_symbolicDecomposition->lnz << " by " */
    /*   << (_blockOrdering ? "block" : "scalar") << " AMD ordering " <<
     * std::endl; */
  }

  void fillCSparse(const SparseBlockMatrix<MatrixType>& A, bool onlyValues) {
    if (!onlyValues) this->initMatrixStructure(A);
    int m = A.rows();
    int n = A.cols();
    assert(m > 0 && n > 0 && "Hessian has 0 rows/cols");

    if (ccsA_.columnsAllocated < n) {
      // pre-allocate more space if re-allocating
      ccsA_.columnsAllocated = ccsA_.columnsAllocated == 0 ? n : 2 * n;
      delete[] ccsA_.p;
      ccsA_.p = new int[ccsA_.columnsAllocated + 1];
    }

    if (!onlyValues) {
      int nzmax = A.nonZeros();
      if (ccsA_.nzmax < nzmax) {
        // pre-allocate more space if re-allocating
        ccsA_.nzmax = ccsA_.nzmax == 0 ? nzmax : 2 * nzmax;
        delete[] ccsA_.x;
        delete[] ccsA_.i;
        ccsA_.i = new int[ccsA_.nzmax];
        ccsA_.x = new number_t[ccsA_.nzmax];
      }
    }
    ccsA_.m = m;
    ccsA_.n = n;

    if (onlyValues) {
      this->ccsMatrix_->fillCCS(ccsA_.x, true);
    } else {
      int nz = this->ccsMatrix_->fillCCS(ccsA_.p, ccsA_.i, ccsA_.x, true);
      (void)nz;
      assert(nz <= ccsA_.nzmax);
    }
    ccsA_.nz = -1;  // tag as CCS formatted matrix
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
    bool ok = true;
    csn* numericCholesky = csparse_extension::cs_chol_workspace(
        &ccsA_, symbolicDecomposition_, csIntWorkspace_, csWorkspace_);
    if (numericCholesky) {
      MarginalCovarianceCholesky mcc;
      mcc.setCholeskyFactor(ccsA_.n, numericCholesky->L->p,
                            numericCholesky->L->i, numericCholesky->L->x,
                            symbolicDecomposition_->pinv);
      compute(mcc);
      cs_nfree(numericCholesky);
    } else {
      ok = false;
      std::cerr << "inverse fail (numeric decomposition)" << std::endl;
    }
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->choleskyNNZ =
          static_cast<size_t>(symbolicDecomposition_->lnz);
    }
    return ok;
  }
};

}  // namespace g2o

#endif
