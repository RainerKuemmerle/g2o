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

#ifndef G2O_LINEAR_SOLVER_PCG_H
#define G2O_LINEAR_SOLVER_PCG_H

#include <Eigen/Core>
#include <utility>
#include <vector>

#include "g2o/core/batch_stats.h"
#include "g2o/core/linear_solver.h"

namespace g2o {

/**
 * \brief linear solver using PCG, pre-conditioner is block Jacobi
 */
template <typename MatrixType>
class LinearSolverPCG : public LinearSolver<MatrixType> {
 public:
  LinearSolverPCG() : LinearSolver<MatrixType>() {
    _tolerance = cst(1e-6);
    _absoluteTolerance = true;
    _residual = -1.0;
    _maxIter = -1;
  }

  virtual ~LinearSolverPCG() {}

  virtual bool init() {
    _residual = -1.0;
    _indices.clear();
    _sparseMat.clear();
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b);

  //! return the tolerance for terminating PCG before convergence
  double tolerance() const { return _tolerance; }
  void setTolerance(double tolerance) { _tolerance = tolerance; }

  int maxIterations() const { return _maxIter; }
  void setMaxIterations(int maxIter) { _maxIter = maxIter; }

  bool absoluteTolerance() const { return _absoluteTolerance; }
  void setAbsoluteTolerance(bool absoluteTolerance) {
    _absoluteTolerance = absoluteTolerance;
  }

 protected:
  using MatrixVector = std::vector<MatrixType>;
  using MatrixPtrVector = std::vector<const MatrixType*>;

  double _tolerance;
  double _residual;
  bool _absoluteTolerance;
  int _maxIter;

  MatrixPtrVector _diag;
  MatrixVector _J;

  std::vector<std::pair<int, int> > _indices;
  MatrixPtrVector _sparseMat;

  void multDiag(const std::vector<int>& colBlockIndices, MatrixVector& A,
                const VectorX& src, VectorX& dest);
  void multDiag(const std::vector<int>& colBlockIndices, MatrixPtrVector& A,
                const VectorX& src, VectorX& dest);
  void mult(const std::vector<int>& colBlockIndices, const VectorX& src,
            VectorX& dest);
};

#include "linear_solver_pcg.hpp"

}  // namespace g2o

#endif
