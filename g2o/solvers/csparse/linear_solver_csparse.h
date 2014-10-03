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

#include "csparse_helper.h"

#include "g2o/core/linear_solver.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/marginal_covariance_cholesky.h"
#include "g2o/stuff/timeutil.h"
#include "g2o_csparse_api.h"

#include <iostream>

namespace g2o {

/**
 * \brief Our C++ version of the csparse struct
 */
struct G2O_SOLVER_CSPARSE_API CSparseExt : public cs
{
  CSparseExt()
  {
    nzmax = 0;
    m = 0;
    n = 0;
    p = 0;
    i = 0;
    x = 0;
    nz = 0;
    columnsAllocated = 0;
  }
  ~CSparseExt()
  {
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
class LinearSolverCSparse : public LinearSolverCCS<MatrixType>
{
  public:
    LinearSolverCSparse() :
      LinearSolverCCS<MatrixType>()
    {
      _symbolicDecomposition = 0;
      _csWorkspaceSize = -1;
      _csWorkspace = 0;
      _csIntWorkspace = 0;
      _ccsA = new CSparseExt;
      _blockOrdering = true;
      _writeDebug = true;
    }

    virtual ~LinearSolverCSparse()
    {
      if (_symbolicDecomposition) {
        cs_sfree(_symbolicDecomposition);
        _symbolicDecomposition = 0;
      }
      delete[] _csWorkspace; _csWorkspace = 0;
      delete[] _csIntWorkspace; _csIntWorkspace = 0;
      delete _ccsA;
    }

    virtual bool init()
    {
      if (_symbolicDecomposition) {
        cs_sfree(_symbolicDecomposition);
        _symbolicDecomposition = 0;
      }
      return true;
    }

    bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
    {
      fillCSparse(A, _symbolicDecomposition != 0);
      // perform symbolic cholesky once
      if (_symbolicDecomposition == 0) {
        computeSymbolicDecomposition(A);
      }
      // re-allocate the temporary workspace for cholesky
      if (_csWorkspaceSize < _ccsA->n) {
        _csWorkspaceSize = 2 * _ccsA->n;
        delete[] _csWorkspace;
        _csWorkspace = new double[_csWorkspaceSize];
        delete[] _csIntWorkspace;
        _csIntWorkspace = new int[2*_csWorkspaceSize];
      }

      double t=get_monotonic_time();
      // _x = _b for calling csparse
      if (x != b)
        memcpy(x, b, _ccsA->n * sizeof(double));
      int ok = csparse_extension::cs_cholsolsymb(_ccsA, x, _symbolicDecomposition, _csWorkspace, _csIntWorkspace);
      if (! ok) {
        if (_writeDebug) {
          std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by Octave)" << std::endl;
          csparse_extension::writeCs2Octave("debug.txt", _ccsA, true);
        }
        return false;
      }

      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats){
        globalStats->timeNumericDecomposition = get_monotonic_time() - t;
        globalStats->choleskyNNZ = static_cast<size_t>(_symbolicDecomposition->lnz);
      }

      return ok != 0;
    }

    bool solveBlocks(double**& blocks, const SparseBlockMatrix<MatrixType>& A) {
      fillCSparse(A, _symbolicDecomposition != 0);
      // perform symbolic cholesky once
      if (_symbolicDecomposition == 0) {
        computeSymbolicDecomposition(A);
        assert(_symbolicDecomposition && "Symbolic cholesky failed");
      }
      // re-allocate the temporary workspace for cholesky
      if (_csWorkspaceSize < _ccsA->n) {
        _csWorkspaceSize = 2 * _ccsA->n;
        delete[] _csWorkspace;
        _csWorkspace = new double[_csWorkspaceSize];
        delete[] _csIntWorkspace;
        _csIntWorkspace = new int[2*_csWorkspaceSize];
      }

      if (! blocks){
        blocks=new double*[A.rows()];
        double **block=blocks;
        for (size_t i=0; i < A.rowBlockIndices().size(); ++i){
          int dim = A.rowsOfBlock(i) * A.colsOfBlock(i);
          *block = new double [dim];
          block++;
        }
      }

      int ok = 1;
      csn* numericCholesky = csparse_extension::cs_chol_workspace(_ccsA, _symbolicDecomposition, _csIntWorkspace, _csWorkspace);
      if (numericCholesky) {
        MarginalCovarianceCholesky mcc;
        mcc.setCholeskyFactor(_ccsA->n, numericCholesky->L->p, numericCholesky->L->i, numericCholesky->L->x, _symbolicDecomposition->pinv);
        mcc.computeCovariance(blocks, A.rowBlockIndices());
        cs_nfree(numericCholesky);
      } else {
        ok = 0;
        std::cerr << "inverse fail (numeric decomposition)" << std::endl;
      }

      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats){
        globalStats->choleskyNNZ = static_cast<size_t>(_symbolicDecomposition->lnz);
      }

      return ok != 0;
    }

    virtual bool solvePattern(SparseBlockMatrix<MatrixXD>& spinv, const std::vector<std::pair<int, int> >& blockIndices, const SparseBlockMatrix<MatrixType>& A) {
      fillCSparse(A, _symbolicDecomposition != 0);
      // perform symbolic cholesky once
      if (_symbolicDecomposition == 0) {
        computeSymbolicDecomposition(A);
        assert(_symbolicDecomposition && "Symbolic cholesky failed");
      }
      // re-allocate the temporary workspace for cholesky
      if (_csWorkspaceSize < _ccsA->n) {
        _csWorkspaceSize = 2 * _ccsA->n;
        delete[] _csWorkspace;
        _csWorkspace = new double[_csWorkspaceSize];
        delete[] _csIntWorkspace;
        _csIntWorkspace = new int[2*_csWorkspaceSize];
      }


      int ok = 1;
      csn* numericCholesky = csparse_extension::cs_chol_workspace(_ccsA, _symbolicDecomposition, _csIntWorkspace, _csWorkspace);
      if (numericCholesky) {
        MarginalCovarianceCholesky mcc;
        mcc.setCholeskyFactor(_ccsA->n, numericCholesky->L->p, numericCholesky->L->i, numericCholesky->L->x, _symbolicDecomposition->pinv);
        mcc.computeCovariance(spinv, A.rowBlockIndices(), blockIndices);
        cs_nfree(numericCholesky);
      } else {
        ok = 0;
        std::cerr << "inverse fail (numeric decomposition)" << std::endl;
      }

      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats){
        globalStats->choleskyNNZ = static_cast<size_t>(_symbolicDecomposition->lnz);
      }

      return ok != 0;
    }

    //! do the AMD ordering on the blocks or on the scalar matrix
    bool blockOrdering() const { return _blockOrdering;}
    void setBlockOrdering(bool blockOrdering) { _blockOrdering = blockOrdering;}

    //! write a debug dump of the system matrix if it is not SPD in solve
    virtual bool writeDebug() const { return _writeDebug;}
    virtual void setWriteDebug(bool b) { _writeDebug = b;}

  protected:
    css* _symbolicDecomposition;
    int _csWorkspaceSize;
    double* _csWorkspace;
    int* _csIntWorkspace;
    CSparseExt* _ccsA;
    bool _blockOrdering;
    MatrixStructure _matrixStructure;
    VectorXI _scalarPermutation;
    bool _writeDebug;

    void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A)
    {
      double t=get_monotonic_time();
      if (! _blockOrdering) {
        _symbolicDecomposition = cs_schol (1, _ccsA) ;
      } else {
        A.fillBlockStructure(_matrixStructure);

        // prepare block structure for the CSparse call
        cs auxBlock;
        auxBlock.nzmax = _matrixStructure.nzMax();
        auxBlock.m = auxBlock.n = _matrixStructure.n;
        auxBlock.p = _matrixStructure.Ap;
        auxBlock.i = _matrixStructure.Aii;
        auxBlock.x = NULL; // no values
        auxBlock.nz = -1; // CCS format

        // AMD ordering on the block structure
        const int& n = _ccsA->n;
        int* P = cs_amd(1, &auxBlock);

        // blow up the permutation to the scalar matrix
        if (_scalarPermutation.size() == 0)
          _scalarPermutation.resize(n);
        if (_scalarPermutation.size() < n)
          _scalarPermutation.resize(2*n);
        size_t scalarIdx = 0;
        for (int i = 0; i < _matrixStructure.n; ++i) {
          const int& p = P[i];
          int base  = A.colBaseOfBlock(p);
          int nCols = A.colsOfBlock(p);
          for (int j = 0; j < nCols; ++j)
            _scalarPermutation(scalarIdx++) = base++;
        }
        assert((int)scalarIdx == n);
        cs_free(P);

        // apply the scalar permutation to finish symbolic decomposition
        _symbolicDecomposition = (css*) cs_calloc(1, sizeof(css));       /* allocate result S */
        _symbolicDecomposition->pinv = cs_pinv(_scalarPermutation.data(), n);
        cs* C = cs_symperm(_ccsA, _symbolicDecomposition->pinv, 0);
        _symbolicDecomposition->parent = cs_etree(C, 0);
        int* post = cs_post(_symbolicDecomposition->parent, n);
        int* c = cs_counts(C, _symbolicDecomposition->parent, post, 0);
        cs_free(post);
        cs_spfree(C);
        _symbolicDecomposition->cp = (int*) cs_malloc(n+1, sizeof(int));
        _symbolicDecomposition->unz = _symbolicDecomposition->lnz = cs_cumsum(_symbolicDecomposition->cp, c, n);
        cs_free(c);
        if (_symbolicDecomposition->lnz < 0) {
          cs_sfree(_symbolicDecomposition);
          _symbolicDecomposition = 0;
        }

      }
      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats){
        globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
      }

      /* std::cerr << "# Number of nonzeros in L: " << (int)_symbolicDecomposition->lnz << " by " */
      /*   << (_blockOrdering ? "block" : "scalar") << " AMD ordering " << std::endl; */
    }

    void fillCSparse(const SparseBlockMatrix<MatrixType>& A, bool onlyValues)
    {
      if (! onlyValues)
        this->initMatrixStructure(A);
      int m = A.rows();
      int n = A.cols();
      assert(m > 0 && n > 0 && "Hessian has 0 rows/cols");

      if (_ccsA->columnsAllocated < n) {
        _ccsA->columnsAllocated = _ccsA->columnsAllocated == 0 ? n : 2 * n; // pre-allocate more space if re-allocating
        delete[] _ccsA->p;
        _ccsA->p = new int[_ccsA->columnsAllocated+1];
      }

      if (! onlyValues) {
        int nzmax = A.nonZeros();
        if (_ccsA->nzmax < nzmax) {
          _ccsA->nzmax = _ccsA->nzmax == 0 ? nzmax : 2 * nzmax; // pre-allocate more space if re-allocating
          delete[] _ccsA->x;
          delete[] _ccsA->i;
          _ccsA->i = new int[_ccsA->nzmax];
          _ccsA->x = new double[_ccsA->nzmax];
        }
      }
      _ccsA->m = m;
      _ccsA->n = n;

      if (onlyValues) {
        this->_ccsMatrix->fillCCS(_ccsA->x, true);
      } else {
        int nz = this->_ccsMatrix->fillCCS(_ccsA->p, _ccsA->i, _ccsA->x, true); (void) nz;
        assert(nz <= _ccsA->nzmax);
      }
      _ccsA->nz=-1; // tag as CCS formatted matrix
    }
};

} // end namespace

#endif
