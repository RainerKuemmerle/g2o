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

#include "g2o/core/linear_solver.h"
#include "g2o/core/batch_stats.h"
#include "g2o/stuff/timeutil.h"

#include <iostream>
#include <vector>

namespace g2o {

/**
 * \brief linear solver which uses the sparse Cholesky solver from Eigen
 *
 * Has no dependencies except Eigen. Hence, should compile almost everywhere
 * without to much issues. Performance should be similar to CSparse, I guess.
 */
template <typename MatrixType>
class LinearSolverEigen: public LinearSolverCCS<MatrixType>
{
  public:
    typedef Eigen::SparseMatrix<number_t, Eigen::ColMajor> SparseMatrix;
    typedef Eigen::Triplet<number_t> Triplet;
    typedef Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> PermutationMatrix;

    using CholeskyDecompositionBase = Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>;

    /**
     * \brief Sub-classing Eigen's SimplicialLDLT to perform ordering with a given ordering
     */
    class CholeskyDecomposition : public CholeskyDecompositionBase
    {
      public:
        CholeskyDecomposition() : CholeskyDecompositionBase() {}
        using CholeskyDecompositionBase::analyzePattern_preordered;

        void analyzePatternWithPermutation(SparseMatrix& a, const PermutationMatrix& permutation)
        {
          m_Pinv = permutation;
          m_P = permutation.inverse();
          int size = a.cols();
          SparseMatrix ap(size, size);
          ap.selfadjointView<Eigen::Upper>() = a.selfadjointView<UpLo>().twistedBy(m_P);
          analyzePattern_preordered(ap, false);
        }
    };

  public:
    LinearSolverEigen() :
      LinearSolverCCS<MatrixType>(),
      _init(true), _blockOrdering(false), _writeDebug(false)
    {
    }

    virtual bool init()
    {
      _init = true;
      return true;
    }

    bool solve(const SparseBlockMatrix<MatrixType>& A, number_t* x, number_t* b)
    {
      double t;
      bool cholState = computeCholesky(A, t);
      if (!cholState) return false;

      // Solving the system
      VectorX::MapType xx(x, _sparseMatrix.cols());
      VectorX::ConstMapType bb(b, _sparseMatrix.cols());
      xx = _cholesky.solve(bb);
      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats) {
        globalStats->timeNumericDecomposition = get_monotonic_time() - t;
        globalStats->choleskyNNZ = _cholesky.matrixL().nestedExpression().nonZeros();
      }
      return true;
    }

    // TODO implement solveBlocks,
    // _cholesky.matrixL().nestedExpression().outerIndexPtr();
    // _cholesky.matrixL().nestedExpression().innerIndexPtr();
    // _cholesky.matrixL().nestedExpression().innerNonZeroPtr();
    // TODO implement solvePattern

    //! do the AMD ordering on the blocks or on the scalar matrix
    bool blockOrdering() const { return _blockOrdering;}
    void setBlockOrdering(bool blockOrdering) { _blockOrdering = blockOrdering;}

    //! write a debug dump of the system matrix if it is not SPD in solve
    virtual bool writeDebug() const { return _writeDebug;}
    virtual void setWriteDebug(bool b) { _writeDebug = b;}

  protected:
    bool _init;
    bool _blockOrdering;
    bool _writeDebug;
    SparseMatrix _sparseMatrix;
    CholeskyDecomposition _cholesky;

    // compute the cholesky factor
    bool computeCholesky(const SparseBlockMatrix<MatrixType>& A, double& t) {
      // perform some operations only once upon init
      if (_init) _sparseMatrix.resize(A.rows(), A.cols());
      fillSparseMatrix(A, !_init);
      if (_init) computeSymbolicDecomposition(A);
      _init = false;

      t = get_monotonic_time();
      _cholesky.factorize(_sparseMatrix);
      if (_cholesky.info() != Eigen::Success) {  // the matrix is not positive definite
        if (_writeDebug) {
          std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by Octave)"
                    << std::endl;
          A.writeOctave("debug.txt");
        }
        return false;
      }
      return true;
    }

    /**
     * compute the symbolic decompostion of the matrix only once.
     * Since A has the same pattern in all the iterations, we only
     * compute the fill-in reducing ordering once and re-use for all
     * the following iterations.
     */
    void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A)
    {
      number_t t=get_monotonic_time();
      if (! _blockOrdering) {
        _cholesky.analyzePattern(_sparseMatrix);
      } else {
        // block ordering with the Eigen Interface
        Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic> blockP;
        {
          SparseMatrix auxBlockMatrix(A.blockCols().size(), A.blockCols().size());
          auxBlockMatrix.resizeNonZeros(A.nonZeroBlocks());

          // fill the CCS structure of the Eigen SparseMatrix
          int nz = 0;
          auto Cp = auxBlockMatrix.outerIndexPtr();
          auto Ci = auxBlockMatrix.innerIndexPtr();
          for (int c = 0; c < static_cast<int>(A.blockCols().size()); ++c) {
            *Cp = nz;
            for (auto it = A.blockCols()[c].begin(); it != A.blockCols()[c].end(); ++it) {
              const int& r = it->first;
              if (r <= c) {
                *Ci++ = r;
                ++nz;
              }
            }
            Cp++;
          }
          *Cp = nz;
          assert(nz <= static_cast<int>(auxBlockMatrix.data().size()));

          using Ordering = Eigen::AMDOrdering<typename CholeskyDecomposition::StorageIndex>;
          Ordering ordering;
          ordering(auxBlockMatrix, blockP);
        }

        int rows = A.rows();
        assert(rows == A.cols() && "Matrix A is not square");

        // Adapt the block permutation to the scalar matrix
        PermutationMatrix scalarP;
        scalarP.resize(rows);
        this->blockToScalarPermutation(A, blockP.indices(), scalarP.indices());
        // analyze with the scalar permutation
        _cholesky.analyzePatternWithPermutation(_sparseMatrix, scalarP);

      }
      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats)
        globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
    }

    void fillSparseMatrix(const SparseBlockMatrix<MatrixType>& A, bool onlyValues) {
      if (onlyValues) {
        this->_ccsMatrix->fillCCS(_sparseMatrix.valuePtr(), true);
        return;
      }
      this->initMatrixStructure(A);
      _sparseMatrix.resizeNonZeros(A.nonZeros());
      int nz =
          this->_ccsMatrix->fillCCS(_sparseMatrix.outerIndexPtr(), _sparseMatrix.innerIndexPtr(),
                                    _sparseMatrix.valuePtr(), true);
      (void)nz;
      assert(nz <= static_cast<int>(_sparseMatrix.data().size()));
    }
};

} // end namespace

#endif
