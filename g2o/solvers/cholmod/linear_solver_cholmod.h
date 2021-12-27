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

#ifndef G2O_LINEAR_SOLVER_CHOLMOD
#define G2O_LINEAR_SOLVER_CHOLMOD

#include <cholmod.h>

#include "g2o/core/batch_stats.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/marginal_covariance_cholesky.h"
#include "g2o/stuff/sparse_helper.h"
#include "g2o/stuff/timeutil.h"

namespace g2o {

/**
 * \brief Our extension of the CHOLMOD matrix struct
 */
struct CholmodExt : public cholmod_sparse {
  CholmodExt() {
    nzmax = 0;
    nrow = 0;
    ncol = 0;
    p = nullptr;
    i = nullptr;
    nz = nullptr;
    x = nullptr;
    z = nullptr;
    stype = 1;  // upper triangular block only
    itype = CHOLMOD_INT;
    xtype = CHOLMOD_REAL;
    dtype = CHOLMOD_DOUBLE;
    sorted = 1;
    packed = 1;
    columnsAllocated = 0;
  }
  ~CholmodExt() {
    delete[] static_cast<int*>(p);
    p = nullptr;
    delete[] static_cast<double*>(x);
    x = nullptr;
    delete[] static_cast<int*>(i);
    i = nullptr;
  }
  size_t columnsAllocated;
};

/**
 * \brief basic solver for Ax = b which has to reimplemented for different
 * linear algebra libraries
 */
template <typename MatrixType>
class LinearSolverCholmod : public LinearSolverCCS<MatrixType> {
 public:
  LinearSolverCholmod() : LinearSolverCCS<MatrixType>() {
    cholmod_start(&cholmodCommon_);

    // setup ordering strategy
    cholmodCommon_.nmethods = 1;
    cholmodCommon_.method[0].ordering = CHOLMOD_AMD;  // CHOLMOD_COLAMD
    //_cholmodCommon.postorder = 0;

    cholmodCommon_.supernodal =
        CHOLMOD_AUTO;  // CHOLMOD_SUPERNODAL; //CHOLMOD_SIMPLICIAL;
  }

  LinearSolverCholmod(LinearSolverCholmod<MatrixType> const&) = delete;
  LinearSolverCholmod& operator=(LinearSolverCholmod<MatrixType> const&) =
      delete;

  ~LinearSolverCholmod() override {
    freeCholdmodFactor();
    cholmod_finish(&cholmodCommon_);
  }

  bool init() override {
    freeCholdmodFactor();
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x,
             double* b) override {
    double t;
    bool cholState = computeCholmodFactor(A, t);
    if (!cholState) return false;

    // setting up b for calling cholmod
    cholmod_dense bcholmod;
    bcholmod.nrow = bcholmod.d = cholmodSparse_.nrow;
    bcholmod.ncol = 1;
    bcholmod.x = b;
    bcholmod.xtype = CHOLMOD_REAL;
    bcholmod.dtype = CHOLMOD_DOUBLE;
    cholmod_dense* xcholmod =
        cholmod_solve(CHOLMOD_A, cholmodFactor_, &bcholmod, &cholmodCommon_);
    memcpy(x, xcholmod->x,
           sizeof(double) * bcholmod.nrow);  // copy back to our array
    cholmod_free_dense(&xcholmod, &cholmodCommon_);

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeNumericDecomposition = get_monotonic_time() - t;
      globalStats->choleskyNNZ =
          static_cast<size_t>(cholmodCommon_.method[0].lnz);
    }

    return true;
  }

  virtual bool saveMatrix(const std::string& fileName) {
    writeCCSMatrix(fileName, cholmodSparse_.nrow, cholmodSparse_.ncol,
                   static_cast<int*>(cholmodSparse_.p),
                   static_cast<int*>(cholmodSparse_.i),
                   static_cast<double*>(cholmodSparse_.x), true);
    return true;
  }

 protected:
  // temp used for cholesky with cholmod
  cholmod_common cholmodCommon_;
  CholmodExt cholmodSparse_;
  cholmod_factor* cholmodFactor_ = nullptr;
  MatrixStructure matrixStructure_;
  VectorXI scalarPermutation_, blockPermutation_;

  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    double t = get_monotonic_time();
    if (!this->blockOrdering()) {
      // setup ordering strategy
      cholmodCommon_.nmethods = 1;
      cholmodCommon_.method[0].ordering = CHOLMOD_AMD;  // CHOLMOD_COLAMD
      cholmodFactor_ = cholmod_analyze(
          &cholmodSparse_, &cholmodCommon_);  // symbolic factorization
    } else {
      A.fillBlockStructure(matrixStructure_);

      // get the ordering for the block matrix
      if (blockPermutation_.size() == 0)
        blockPermutation_.resize(matrixStructure_.n);
      if (blockPermutation_.size() <
          matrixStructure_.n)  // double space if resizing
        blockPermutation_.resize(2L * matrixStructure_.n);

      // prepare AMD call via CHOLMOD
      cholmod_sparse auxCholmodSparse;
      auxCholmodSparse.nzmax = matrixStructure_.nzMax();
      auxCholmodSparse.nrow = auxCholmodSparse.ncol = matrixStructure_.n;
      auxCholmodSparse.p = matrixStructure_.Ap;
      auxCholmodSparse.i = matrixStructure_.Aii;
      auxCholmodSparse.nz = nullptr;
      auxCholmodSparse.x = nullptr;
      auxCholmodSparse.z = nullptr;
      auxCholmodSparse.stype = 1;
      auxCholmodSparse.xtype = CHOLMOD_PATTERN;
      auxCholmodSparse.itype = CHOLMOD_INT;
      auxCholmodSparse.dtype = CHOLMOD_DOUBLE;
      auxCholmodSparse.sorted = 1;
      auxCholmodSparse.packed = 1;
      int amdStatus = cholmod_amd(&auxCholmodSparse, nullptr, 0,
                                  blockPermutation_.data(), &cholmodCommon_);
      if (!amdStatus) return;

      // blow up the permutation to the scalar matrix
      this->blockToScalarPermutation(A, blockPermutation_, scalarPermutation_);

      // apply the ordering
      cholmodCommon_.nmethods = 1;
      cholmodCommon_.method[0].ordering = CHOLMOD_GIVEN;
      cholmodFactor_ =
          cholmod_analyze_p(&cholmodSparse_, scalarPermutation_.data(), nullptr,
                            0, &cholmodCommon_);
    }
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats)
      globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
  }

  void fillCholmodExt(const SparseBlockMatrix<MatrixType>& A, bool onlyValues) {
    if (!onlyValues) this->initMatrixStructure(A);
    size_t m = A.rows();
    size_t n = A.cols();
    assert(m > 0 && n > 0 && "Hessian has 0 rows/cols");

    if (cholmodSparse_.columnsAllocated < n) {
      // pre-allocate more space if re-allocating
      cholmodSparse_.columnsAllocated =
          cholmodSparse_.columnsAllocated == 0 ? n : 2 * n;
      delete[] static_cast<int*>(cholmodSparse_.p);
      cholmodSparse_.p = new int[cholmodSparse_.columnsAllocated + 1];
    }
    if (!onlyValues) {
      size_t nzmax = A.nonZeros();
      if (cholmodSparse_.nzmax < nzmax) {
        // pre-allocate more space if re-allocating
        cholmodSparse_.nzmax = cholmodSparse_.nzmax == 0 ? nzmax : 2 * nzmax;
        delete[] static_cast<double*>(cholmodSparse_.x);
        delete[] static_cast<int*>(cholmodSparse_.i);
        cholmodSparse_.i = new int[cholmodSparse_.nzmax];
        cholmodSparse_.x = new double[cholmodSparse_.nzmax];
      }
    }
    cholmodSparse_.ncol = n;
    cholmodSparse_.nrow = m;

    if (onlyValues)
      this->ccsMatrix_->fillCCS(static_cast<double*>(cholmodSparse_.x), true);
    else
      this->ccsMatrix_->fillCCS(static_cast<int*>(cholmodSparse_.p),
                                static_cast<int*>(cholmodSparse_.i),
                                static_cast<double*>(cholmodSparse_.x), true);
  }

  //! compute the cholmodFactor for the given matrix A
  bool computeCholmodFactor(const SparseBlockMatrix<MatrixType>& A, double& t) {
    // _cholmodFactor used as bool, if not existing will copy the whole
    // structure, otherwise only the values
    fillCholmodExt(A, cholmodFactor_ != nullptr);

    if (cholmodFactor_ == nullptr) {
      computeSymbolicDecomposition(A);
      assert(_cholmodFactor != 0 && "Symbolic cholesky failed");
    }
    t = get_monotonic_time();

    cholmod_factorize(&cholmodSparse_, cholmodFactor_, &cholmodCommon_);
    if (cholmodCommon_.status == CHOLMOD_NOT_POSDEF) {
      if (this->writeDebug()) {
        std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by "
                     "Octave)"
                  << std::endl;
        saveMatrix("debug.txt");
      }
      return false;
    }
    return true;
  }

  bool solveBlocks_impl(const SparseBlockMatrix<MatrixType>& A,
                        const std::function<void(MarginalCovarianceCholesky&)>&
                            compute) override {
    double t;
    bool cholState = computeCholmodFactor(A, t);
    if (!cholState) return false;

    // convert the factorization to LL, simplical, packed, monotonic
    int change_status = cholmod_change_factor(CHOLMOD_REAL, 1, 0, 1, 1,
                                              cholmodFactor_, &cholmodCommon_);
    if (!change_status) return false;
    assert(_cholmodFactor->is_ll && !_cholmodFactor->is_super &&
           _cholmodFactor->is_monotonic && "Cholesky factor has wrong format");

    // invert the permutation
    int* p = static_cast<int*>(cholmodFactor_->Perm);
    VectorXI pinv(cholmodSparse_.ncol);
    for (size_t i = 0; i < cholmodSparse_.ncol; ++i) pinv(p[i]) = i;

    // compute the marginal covariance
    MarginalCovarianceCholesky mcc;
    mcc.setCholeskyFactor(cholmodSparse_.ncol,
                          static_cast<int*>(cholmodFactor_->p),
                          static_cast<int*>(cholmodFactor_->i),
                          static_cast<double*>(cholmodFactor_->x), pinv.data());
    compute(mcc);

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->choleskyNNZ = static_cast<size_t>(
          cholmodCommon_.method[cholmodCommon_.selected].lnz);
    }
    return true;
  }

  void freeCholdmodFactor() {
    if (cholmodFactor_ != nullptr) {
      cholmod_free_factor(&cholmodFactor_, &cholmodCommon_);
      cholmodFactor_ = nullptr;
    }
  }
};

}  // namespace g2o
#endif
