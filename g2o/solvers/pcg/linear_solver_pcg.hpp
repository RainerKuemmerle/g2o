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

// helpers for doing fixed or variable size operations on the matrices

namespace internal {

#ifdef _MSC_VER
  // MSVC does not like the template specialization, seems like MSVC applies type conversion
  // which results in calling a fixed size method (segment<int>) on the dynamically sized matrices
  template<typename MatrixType>
  void pcg_axy(const MatrixType& A, const VectorXD& x, int xoff, VectorXD& y, int yoff)
  {
    y.segment(yoff, A.rows()) = A * x.segment(xoff, A.cols());
  }
#else
  template<typename MatrixType>
  inline void pcg_axy(const MatrixType& A, const VectorXD& x, int xoff, VectorXD& y, int yoff)
  {
    y.segment<MatrixType::RowsAtCompileTime>(yoff) = A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
  }
  
  template<>
  inline void pcg_axy(const MatrixXD& A, const VectorXD& x, int xoff, VectorXD& y, int yoff)
  {
    y.segment(yoff, A.rows()) = A * x.segment(xoff, A.cols());
  }
#endif

  template<typename MatrixType>
  inline void pcg_axpy(const MatrixType& A, const VectorXD& x, int xoff, VectorXD& y, int yoff)
  {
    y.segment<MatrixType::RowsAtCompileTime>(yoff) += A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
  }

  template<>
  inline void pcg_axpy(const MatrixXD& A, const VectorXD& x, int xoff, VectorXD& y, int yoff)
  {
    y.segment(yoff, A.rows()) += A * x.segment(xoff, A.cols());
  }

  template<typename MatrixType>
  inline void pcg_atxpy(const MatrixType& A, const VectorXD& x, int xoff, VectorXD& y, int yoff)
  {
    y.segment<MatrixType::ColsAtCompileTime>(yoff) += A.transpose() * x.segment<MatrixType::RowsAtCompileTime>(xoff);
  }

  template<>
  inline void pcg_atxpy(const MatrixXD& A, const VectorXD& x, int xoff, VectorXD& y, int yoff)
  {
    y.segment(yoff, A.cols()) += A.transpose() * x.segment(xoff, A.rows());
  }
}
// helpers end

template <typename MatrixType>
bool LinearSolverPCG<MatrixType>::solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
{
  const bool indexRequired = _indices.size() == 0;
  _diag.clear();
  _J.clear();

  // put the block matrix once in a linear structure, makes mult faster
  int colIdx = 0;
  for (size_t i = 0; i < A.blockCols().size(); ++i){
    const typename SparseBlockMatrix<MatrixType>::IntBlockMap& col = A.blockCols()[i];
    if (col.size() > 0) {
      typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it;
      for (it = col.begin(); it != col.end(); ++it) {
        if (it->first == (int)i) { // only the upper triangular block is needed
          _diag.push_back(it->second);
          _J.push_back(it->second->inverse());
          break;
        }
        if (indexRequired) {
          _indices.push_back(std::make_pair(it->first > 0 ? A.rowBlockIndices()[it->first-1] : 0, colIdx));
          _sparseMat.push_back(it->second);
        }

      }
    }
    colIdx = A.colBlockIndices()[i];
  }

  int n = A.rows();
  assert(n > 0 && "Hessian has 0 rows/cols");
  Eigen::Map<VectorXD> xvec(x, A.cols());
  const Eigen::Map<VectorXD> bvec(b, n);
  xvec.setZero();

  VectorXD r, d, q, s;
  d.setZero(n);
  q.setZero(n);
  s.setZero(n);

  r = bvec;
  multDiag(A.colBlockIndices(), _J, r, d);
  double dn = r.dot(d);
  double d0 = _tolerance * dn;

  if (_absoluteTolerance) {
    if (_residual > 0.0 && _residual > d0)
      d0 = _residual;
  }

  int maxIter = _maxIter < 0 ? A.rows() : _maxIter;

  int iteration;
  for (iteration = 0; iteration < maxIter; ++iteration) {
    if (_verbose)
      std::cerr << "residual[" << iteration << "]: " << dn << std::endl;
    if (dn <= d0)
      break;  // done
    mult(A.colBlockIndices(), d, q);
    double a = dn / d.dot(q);
    xvec += a*d;
    // TODO: reset residual here every 50 iterations
    r -= a*q;
    multDiag(A.colBlockIndices(), _J, r, s);
    double dold = dn;
    dn = r.dot(s);
    double ba = dn / dold;
    d = s + ba*d;
  }
  //std::cerr << "residual[" << iteration << "]: " << dn << std::endl;
  _residual = 0.5 * dn;
  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats) {
    globalStats->iterationsLinearSolver = iteration;
  }

  return true;
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::multDiag(const std::vector<int>& colBlockIndices, MatrixVector& A, const VectorXD& src, VectorXD& dest)
{
  int row = 0;
  for (size_t i = 0; i < A.size(); ++i) {
    internal::pcg_axy(A[i], src, row, dest, row);
    row = colBlockIndices[i];
  }
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::multDiag(const std::vector<int>& colBlockIndices, MatrixPtrVector& A, const VectorXD& src, VectorXD& dest)
{
  int row = 0;
  for (size_t i = 0; i < A.size(); ++i) {
    internal::pcg_axy(*A[i], src, row, dest, row);
    row = colBlockIndices[i];
  }
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::mult(const std::vector<int>& colBlockIndices, const VectorXD& src, VectorXD& dest)
{
  // first multiply with the diagonal
  multDiag(colBlockIndices, _diag, src, dest);

  // now multiply with the upper triangular block
  for (size_t i = 0; i < _sparseMat.size(); ++i) {
    const int& srcOffset = _indices[i].second;
    const int& destOffsetT = srcOffset;
    const int& destOffset = _indices[i].first;
    const int& srcOffsetT = destOffset;

    const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a = _sparseMat[i];
    // destVec += *a * srcVec (according to the sub-vector parts)
    internal::pcg_axpy(*a, src, srcOffset, dest, destOffset);
    // destVec += *a.transpose() * srcVec (according to the sub-vector parts)
    internal::pcg_atxpy(*a, src, srcOffsetT, dest, destOffsetT);
  }
}
