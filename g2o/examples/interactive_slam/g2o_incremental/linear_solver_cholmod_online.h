// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_LINEAR_SOLVER_CHOLMOD_ONLINE
#define G2O_LINEAR_SOLVER_CHOLMOD_ONLINE

#include <camd.h>
#include <cholmod.h>

#include "g2o/core/batch_stats.h"
#include "g2o/core/linear_solver.h"
#include "g2o/solvers/cholmod/cholmod_ext.h"
#include "g2o/stuff/timeutil.h"
#include "g2o_incremental_api.h"

namespace g2o {

/**
 * \brief generic interface for the online solver
 */
class G2O_INCREMENTAL_API LinearSolverCholmodOnlineInterface {
 public:
  LinearSolverCholmodOnlineInterface() = default;
  virtual int choleskyUpdate(cholmod_sparse* update) = 0;
  virtual bool solve(double* x, double* b) = 0;
  virtual cholmod_factor* L() const = 0;
  virtual size_t nonZerosInL() const = 0;
  Eigen::VectorXi* cmember = nullptr;
  int batchEveryN = 100;
};

/**
 * \brief linear solver which allows to update the cholesky factor
 */
template <typename MatrixType>
class LinearSolverCholmodOnline : public LinearSolver<MatrixType>,
                                  public LinearSolverCholmodOnlineInterface {
 public:
  LinearSolverCholmodOnline() : LinearSolver<MatrixType>() {
    cholmodSparse_ = new cholmod::CholmodExt();
    cholmodFactor_ = nullptr;
    cholmod_start(&cholmodCommon_);

    // setup ordering strategy
    cholmodCommon_.nmethods = 1;
    cholmodCommon_.method[0].ordering = CHOLMOD_AMD;  // CHOLMOD_COLAMD
    // cholmodCommon_.postorder = 0;

    cholmodCommon_.supernodal =
        CHOLMOD_AUTO;  // CHOLMOD_SUPERNODAL; //CHOLMOD_SIMPLICIAL;
    batchEveryN = 100;
  }

  ~LinearSolverCholmodOnline() override {
    delete cholmodSparse_;
    if (cholmodFactor_) {
      cholmod_free_factor(&cholmodFactor_, &cholmodCommon_);
      cholmodFactor_ = nullptr;
    }
    cholmod_finish(&cholmodCommon_);
  }

  bool init() override { return true; }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x,
             double* b) override {
    cholmod_free_factor(&cholmodFactor_, &cholmodCommon_);
    cholmodFactor_ = nullptr;
    fillCholmodExt(A, false);

    computeSymbolicDecomposition(A);
    assert(cholmodFactor_ && "Symbolic cholesky failed");
    const double t = get_monotonic_time();

    // setting up b for calling cholmod
    cholmod_dense bcholmod;
    bcholmod.nrow = bcholmod.d = cholmodSparse_->nrow;
    bcholmod.ncol = 1;
    bcholmod.x = b;
    bcholmod.xtype = CHOLMOD_REAL;
    bcholmod.dtype = CHOLMOD_DOUBLE;

    cholmod_factorize(cholmodSparse_, cholmodFactor_, &cholmodCommon_);
    if (cholmodCommon_.status == CHOLMOD_NOT_POSDEF) {
      std::cerr << "solve(): Cholesky failure, writing debug.txt (Hessian "
                   "loadable by Octave)"
                << std::endl;
      writeCCSMatrix("debug.txt", cholmodSparse_->nrow, cholmodSparse_->ncol,
                     reinterpret_cast<int*>(cholmodSparse_->p),
                     reinterpret_cast<int*>(cholmodSparse_->i),
                     reinterpret_cast<double*>(cholmodSparse_->x), true);
      return false;
    }

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

  bool blockOrdering() const { return true; }

  cholmod_factor* L() const override { return cholmodFactor_; }

  /**
   * return the number of non-zeros in the current factorization
   */
  size_t nonZerosInL() const override {
    size_t nnz = 0;
    int* nz = reinterpret_cast<int*>(cholmodFactor_->nz);
    if (!nz) return 0;
    for (size_t i = 0; i < cholmodFactor_->n; ++i) nnz += nz[i];
    return nnz;
  }

  int choleskyUpdate(cholmod_sparse* update) override {
    int result = cholmod_updown(1, update, cholmodFactor_, &cholmodCommon_);
    // std::cerr << __PRETTY_FUNCTION__ << " " << result << std::endl;
    if (cholmodCommon_.status == CHOLMOD_NOT_POSDEF) {
      std::cerr
          << "Cholesky failure, writing debug.txt (Hessian loadable by Octave)"
          << std::endl;
      writeCCSMatrix("debug.txt", cholmodSparse_->nrow, cholmodSparse_->ncol,
                     reinterpret_cast<int*>(cholmodSparse_->p),
                     reinterpret_cast<int*>(cholmodSparse_->i),
                     reinterpret_cast<double*>(cholmodSparse_->x), true);
      return 0;
    }
    return result;
  }

  bool solve(double* x, double* b) override {
    // setting up b for calling cholmod
    cholmod_dense bcholmod;
    bcholmod.nrow = bcholmod.d = cholmodSparse_->nrow;
    bcholmod.ncol = 1;
    bcholmod.x = b;
    bcholmod.xtype = CHOLMOD_REAL;
    bcholmod.dtype = CHOLMOD_DOUBLE;

    cholmod_dense* xcholmod =
        cholmod_solve(CHOLMOD_A, cholmodFactor_, &bcholmod, &cholmodCommon_);
    memcpy(x, xcholmod->x,
           sizeof(double) * bcholmod.nrow);  // copy back to our array
    cholmod_free_dense(&xcholmod, &cholmodCommon_);
    return true;
  }

 protected:
  // temp used for cholesky with cholmod
  cholmod_common cholmodCommon_;
  cholmod::CholmodExt* cholmodSparse_;
  cholmod_factor* cholmodFactor_;
  bool blockOrdering_;
  MatrixStructure matrixStructure_;
  Eigen::VectorXi scalarPermutation_, blockPermutation_;

 public:
  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    const double t = get_monotonic_time();

    A.fillBlockStructure(matrixStructure_);

    // double space if resizing
    if (blockPermutation_.size() < matrixStructure_.n) {
      blockPermutation_.resize(static_cast<Eigen::Index>(2) *
                               matrixStructure_.n);
    }

    // get the ordering for the block matrix
    int amdStatus = camd_order(matrixStructure_.n, matrixStructure_.Ap,
                               matrixStructure_.Aii, blockPermutation_.data(),
                               nullptr, nullptr, cmember->data());
    if (amdStatus != CAMD_OK) {
      std::cerr << "Error while computing ordering" << std::endl;
    }

    // blow up the permutation to the scalar matrix and extend to include the
    // additional blocks
    if (scalarPermutation_.size() == 0)
      scalarPermutation_.resize(cholmodSparse_->ncol);
    if (scalarPermutation_.size() < static_cast<int>(cholmodSparse_->ncol))
      scalarPermutation_.resize(2 * cholmodSparse_->ncol);
    size_t scalarIdx = 0;
    for (int i = 0; i < matrixStructure_.n; ++i) {
      const int& p = blockPermutation_(i);
      int base = A.colBaseOfBlock(p);
      int nCols = A.colsOfBlock(p);
      for (int j = 0; j < nCols; ++j) scalarPermutation_(scalarIdx++) = base++;
    }

    for (; scalarIdx < cholmodSparse_->ncol;
         ++scalarIdx)  // extending for the additional blocks
      scalarPermutation_(scalarIdx) = scalarIdx;
    assert(scalarIdx == cholmodSparse_->ncol);

    // apply the ordering
    cholmodCommon_.nmethods = 1;
    cholmodCommon_.method[0].ordering = CHOLMOD_GIVEN;
    cholmodFactor_ = cholmod_analyze_p(
        cholmodSparse_, scalarPermutation_.data(), nullptr, 0, &cholmodCommon_);

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats)
      globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
  }

  void fillCholmodExt(const SparseBlockMatrix<MatrixType>& A, bool onlyValues) {
    int blockDimension = MatrixType::RowsAtCompileTime;
    assert(blockDimension > 0);
    // size_t origM = A.rows();
    size_t origN = A.cols();
    int additionalSpace = blockDimension * batchEveryN;
    size_t m = A.rows() + additionalSpace;
    size_t n = A.cols() + additionalSpace;

    if (cholmodSparse_->columnsAllocated < n) {
      // std::cerr << __PRETTY_FUNCTION__ << ": reallocating columns" <<
      // std::endl;
      cholmodSparse_->columnsAllocated =
          cholmodSparse_->columnsAllocated == 0
              ? n
              : 2 * n;  // pre-allocate more space if re-allocating
      delete[] reinterpret_cast<int*>(cholmodSparse_->p);
      cholmodSparse_->p = new int[cholmodSparse_->columnsAllocated + 1];
    }
    if (!onlyValues) {
      size_t nzmax = A.nonZeros() + additionalSpace;
      if (cholmodSparse_->nzmax < nzmax) {
        // std::cerr << __PRETTY_FUNCTION__ << ": reallocating row + values" <<
        // std::endl;
        cholmodSparse_->nzmax =
            cholmodSparse_->nzmax == 0
                ? nzmax
                : 2 * nzmax;  // pre-allocate more space if re-allocating
        delete[] reinterpret_cast<double*>(cholmodSparse_->x);
        delete[] reinterpret_cast<int*>(cholmodSparse_->i);
        cholmodSparse_->i = new int[cholmodSparse_->nzmax];
        cholmodSparse_->x = new double[cholmodSparse_->nzmax];
      }
    }
    cholmodSparse_->ncol = n;
    cholmodSparse_->nrow = m;

    int nz = 0;
    if (onlyValues)
      nz = A.fillCCS(reinterpret_cast<double*>(cholmodSparse_->x), true);
    else
      nz = A.fillCCS(reinterpret_cast<int*>(cholmodSparse_->p),
                     reinterpret_cast<int*>(cholmodSparse_->i),
                     reinterpret_cast<double*>(cholmodSparse_->x), true);

    int* cp = reinterpret_cast<int*>(cholmodSparse_->p);
    int* ci = reinterpret_cast<int*>(cholmodSparse_->i);
    auto* cx = reinterpret_cast<double*>(cholmodSparse_->x);

    cp = &cp[origN];
    ci = &ci[nz];
    cx = &cx[nz];

    // diagonal for the next blocks
    for (int i = 0; i < additionalSpace; ++i) {
      *cp++ = nz;
      *ci++ = origN + i;
      *cx++ = 1e-6;
      ++nz;
    }
    *cp = nz;
  }
};

}  // namespace g2o

#endif
