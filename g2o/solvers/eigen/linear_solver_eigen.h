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

#ifndef G2O_LINEAR_SOLVER_EIGEN_H
#define G2O_LINEAR_SOLVER_EIGEN_H

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <iostream>
#include <vector>

#include "g2o/core/batch_stats.h"
#include "g2o/core/linear_solver.h"
#include "g2o/core/marginal_covariance_cholesky.h"
#include "g2o/stuff/timeutil.h"

namespace g2o {

/**
 * \brief linear solver which uses the sparse Cholesky solver from Eigen
 *
 * Has no dependencies except Eigen. Hence, should compile almost everywhere
 * without too much issues. Performance should be similar to CSparse.
 */
template <typename MatrixType>
class LinearSolverEigen : public LinearSolverCCS<MatrixType> {
 public:
  using SparseMatrix = Eigen::SparseMatrix<number_t, Eigen::ColMajor>;
  using Triplet = Eigen::Triplet<number_t>;
  using PermutationMatrix =
      Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic>;

  using CholeskyDecompositionBase =
      Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>;

  /**
   * \brief Sub-classing Eigen's SimplicialLLT to perform ordering with a given
   * ordering
   */
  class CholeskyDecomposition : public CholeskyDecompositionBase {
   public:
    CholeskyDecomposition() : CholeskyDecompositionBase() {}

    //! use a given permutation for analyzing the pattern of the sparse matrix
    void analyzePatternWithPermutation(SparseMatrix& a,
                                       const PermutationMatrix& permutation) {
      m_Pinv = permutation;
      m_P = permutation.inverse();
      const int size = a.cols();
      SparseMatrix ap(size, size);
      ap.selfadjointView<Eigen::Upper>() =
          a.selfadjointView<UpLo>().twistedBy(m_P);
      analyzePattern_preordered(ap, false);
    }

   protected:
    using CholeskyDecompositionBase::analyzePattern_preordered;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  LinearSolverEigen() : LinearSolverCCS<MatrixType>() {}

  bool init() override {
    init_ = true;
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, number_t* x,
             number_t* b) override {
    double t;
    bool cholState = computeCholesky(A, t);
    if (!cholState) return false;

    // Solving the system
    VectorX::MapType xx(x, sparseMatrix_.cols());
    VectorX::ConstMapType bb(b, sparseMatrix_.cols());
    xx = cholesky_.solve(bb);
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->timeNumericDecomposition = get_monotonic_time() - t;
      globalStats->choleskyNNZ =
          cholesky_.matrixL().nestedExpression().nonZeros();
    }
    return true;
  }

 protected:
  bool init_ = true;
  SparseMatrix sparseMatrix_;
  CholeskyDecomposition cholesky_;

  // compute the cholesky factor
  bool computeCholesky(const SparseBlockMatrix<MatrixType>& A, double& t) {
    // perform some operations only once upon init
    if (init_) sparseMatrix_.resize(A.rows(), A.cols());
    fillSparseMatrix(A, !init_);
    if (init_) computeSymbolicDecomposition(A);
    init_ = false;

    t = get_monotonic_time();
    cholesky_.factorize(sparseMatrix_);
    if (cholesky_.info() !=
        Eigen::Success) {  // the matrix is not positive definite
      if (this->writeDebug()) {
        std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by "
                     "Octave)"
                  << std::endl;
        A.writeOctave("debug.txt");
      }
      return false;
    }
    return true;
  }

  /**
   * compute the symbolic decomposition of the matrix only once.
   * Since A has the same pattern in all the iterations, we only
   * compute the fill-in reducing ordering once and re-use for all
   * the following iterations.
   */
  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    const number_t t = get_monotonic_time();
    if (!this->blockOrdering()) {
      cholesky_.analyzePattern(sparseMatrix_);
    } else {
      assert(A.rows() == A.cols() && "Matrix A is not square");
      // block ordering with the Eigen Interface
      Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> blockP;
      {
        SparseMatrix auxBlockMatrix(A.blockCols().size(), A.blockCols().size());
        auxBlockMatrix.resizeNonZeros(A.nonZeroBlocks());
        // fill the CCS structure of the Eigen SparseMatrix
        A.fillBlockStructure(auxBlockMatrix.outerIndexPtr(),
                             auxBlockMatrix.innerIndexPtr());
        // determine ordering by AMD
        using Ordering = Eigen::AMDOrdering<SparseMatrix::StorageIndex>;
        Ordering ordering;
        ordering(auxBlockMatrix, blockP);
      }

      // Adapt the block permutation to the scalar matrix
      PermutationMatrix scalarP(A.rows());
      this->blockToScalarPermutation(A, blockP.indices(), scalarP.indices());
      // analyze with the scalar permutation
      cholesky_.analyzePatternWithPermutation(sparseMatrix_, scalarP);
    }
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats)
      globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
  }

  void fillSparseMatrix(const SparseBlockMatrix<MatrixType>& A,
                        bool onlyValues) {
    if (onlyValues) {
      this->ccsMatrix_->fillCCS(sparseMatrix_.valuePtr(), true);
      return;
    }
    this->initMatrixStructure(A);
    sparseMatrix_.resizeNonZeros(A.nonZeros());
    int nz = this->ccsMatrix_->fillCCS(sparseMatrix_.outerIndexPtr(),
                                       sparseMatrix_.innerIndexPtr(),
                                       sparseMatrix_.valuePtr(), true);
    (void)nz;
    assert(nz <= static_cast<int>(sparseMatrix_.data().size()));
  }

  /**
   * Implementation of the general parts for computing the inverse blocks of the
   * linear system matrix. Here we call a function to do the underlying
   * computation.
   */
  bool solveBlocks_impl(const SparseBlockMatrix<MatrixType>& A,
                        const std::function<void(MarginalCovarianceCholesky&)>&
                            compute) override {
    // compute the cholesky factor
    double t;
    bool cholState = computeCholesky(A, t);
    if (!cholState) return false;
    // compute the inverse blocks
    MarginalCovarianceCholesky mcc;
    mcc.setCholeskyFactor(
        cholesky_.matrixL().rows(),
        const_cast<int*>(
            cholesky_.matrixL().nestedExpression().outerIndexPtr()),
        const_cast<int*>(
            cholesky_.matrixL().nestedExpression().innerIndexPtr()),
        const_cast<number_t*>(
            cholesky_.matrixL().nestedExpression().valuePtr()),
        const_cast<int*>(cholesky_.permutationP().indices().data()));
    compute(mcc);  // call the desired computation on the mcc object
    // book keeping statistics
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats) {
      globalStats->choleskyNNZ =
          cholesky_.matrixL().nestedExpression().nonZeros();
    }
    return true;
  }
};

}  // namespace g2o

#endif
