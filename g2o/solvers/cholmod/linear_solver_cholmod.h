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

#include <cassert>

#include "cholmod_wrapper.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/marginal_covariance_cholesky.h"
#include "g2o/stuff/sparse_helper.h"
#include "g2o/stuff/timeutil.h"

namespace g2o {

/**
 * \brief basic solver for Ax = b which has to reimplemented for different
 * linear algebra libraries
 */
template <typename MatrixType>
class LinearSolverCholmod : public LinearSolverCCS<MatrixType> {
 public:
  LinearSolverCholmod() : LinearSolverCCS<MatrixType>() {}

  LinearSolverCholmod(LinearSolverCholmod<MatrixType> const&) = delete;
  LinearSolverCholmod& operator=(LinearSolverCholmod<MatrixType> const&) =
      delete;

  virtual ~LinearSolverCholmod() { freeCholdmodFactor(); }

  virtual bool init() {
    freeCholdmodFactor();
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) {
    double t;
    bool cholState = computeCholmodFactor(A, t);
    if (!cholState) return false;

    _cholmod.solve(x, b);

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeNumericDecomposition = get_monotonic_time() - t;
      globalStats->choleskyNNZ = _cholmod.choleskyNz();
    }

    return true;
  }

  virtual bool saveMatrix(const std::string& fileName) {
    cholmod::Cholmod::SparseView sparseView = _cholmod.sparseView();
    writeCCSMatrix(fileName, sparseView.nrow, sparseView.ncol, sparseView.p,
                   sparseView.i, sparseView.x, true);
    return true;
  }

 protected:
  // temp used for cholesky with cholmod
  cholmod::Cholmod _cholmod;
  MatrixStructure _matrixStructure;
  VectorXI _scalarPermutation, _blockPermutation;

  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    double t = get_monotonic_time();
    if (!this->blockOrdering()) {
      _cholmod.analyze();
    } else {
      A.fillBlockStructure(_matrixStructure);

      // get the ordering for the block matrix
      if (_blockPermutation.size() == 0)
        _blockPermutation.resize(_matrixStructure.n);
      if (_blockPermutation.size() <
          _matrixStructure.n)  // double space if resizing
        _blockPermutation.resize(2 * _matrixStructure.n);

      // prepare AMD call via CHOLMOD
      size_t structureDim = _matrixStructure.n;
      size_t structureNz = _matrixStructure.nzMax();
      size_t structureAllocated = structureDim;
      double* structureX = nullptr;
      cholmod::Cholmod::SparseView amdView(
          structureDim, structureDim, structureNz, _matrixStructure.Ap,
          _matrixStructure.Aii, structureX, structureAllocated);
      bool amdStatus = _cholmod.amd(amdView, _blockPermutation.data());
      if (!amdStatus) return;

      // blow up the permutation to the scalar matrix
      this->blockToScalarPermutation(A, _blockPermutation, _scalarPermutation);

      // apply the ordering
      _cholmod.analyze_p(_scalarPermutation.data());
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

    cholmod::Cholmod::SparseView cholmodSparse = _cholmod.sparseView();

    if (cholmodSparse.columnsAllocated < n) {
      // pre-allocate more space if re-allocating
      cholmodSparse.columnsAllocated =
          cholmodSparse.columnsAllocated == 0 ? n : 2 * n;
      delete[] cholmodSparse.p;
      cholmodSparse.p = new int[cholmodSparse.columnsAllocated + 1];
    }
    if (!onlyValues) {
      size_t nzmax = A.nonZeros();
      if (cholmodSparse.nzmax < nzmax) {
        // pre-allocate more space if re-allocating
        cholmodSparse.nzmax = cholmodSparse.nzmax == 0 ? nzmax : 2 * nzmax;
        delete[] cholmodSparse.x;
        delete[] cholmodSparse.i;
        cholmodSparse.i = new int[cholmodSparse.nzmax];
        cholmodSparse.x = new double[cholmodSparse.nzmax];
      }
    }
    cholmodSparse.ncol = n;
    cholmodSparse.nrow = m;

    if (onlyValues)
      this->_ccsMatrix->fillCCS((double*)cholmodSparse.x, true);
    else
      this->_ccsMatrix->fillCCS((int*)cholmodSparse.p, (int*)cholmodSparse.i,
                                (double*)cholmodSparse.x, true);
  }

  //! compute the cholmodFactor for the given matrix A
  bool computeCholmodFactor(const SparseBlockMatrix<MatrixType>& A, double& t) {
    // _cholmodFactor used as bool, if not existing will copy the whole
    // structure, otherwise only the values
    bool hasFactor = _cholmod.hasFactor();
    fillCholmodExt(A, hasFactor);

    if (!hasFactor) {
      computeSymbolicDecomposition(A);
      assert(_cholmod.hasFactor() && "Symbolic cholesky failed");
    }

    t = get_monotonic_time();
    bool factorStatus = _cholmod.factorize();
    if (!factorStatus) {
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

  bool solveBlocks_impl(
      const SparseBlockMatrix<MatrixType>& A,
      std::function<void(MarginalCovarianceCholesky&)> compute) {
    double t;
    bool cholState = computeCholmodFactor(A, t);
    if (!cholState) return false;

    // convert the factorization to LL, simplical, packed, monotonic
    int change_status = _cholmod.simplifyFactor();
    if (!change_status) return false;

    cholmod::Cholmod::FactorView cholmodFactor = _cholmod.factor();
    cholmod::Cholmod::SparseView cholmodSparse = _cholmod.sparseView();

    // invert the permutation
    int* p = cholmodFactor.perm;
    VectorXI pinv(cholmodSparse.ncol);
    for (size_t i = 0; i < cholmodSparse.ncol; ++i) pinv(p[i]) = i;

    // compute the marginal covariance
    MarginalCovarianceCholesky mcc;
    mcc.setCholeskyFactor(cholmodSparse.ncol, cholmodFactor.p, cholmodFactor.i,
                          cholmodFactor.x, pinv.data());
    compute(mcc);

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->choleskyNNZ = _cholmod.choleskyNz();
    }
    return true;
  }

  void freeCholdmodFactor() { _cholmod.freeFactor(); }
};

}  // namespace g2o
#endif
