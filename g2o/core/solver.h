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

#ifndef G2O_SOLVER_H
#define G2O_SOLVER_H

#include <cstddef>

#include "g2o_core_api.h"
#include "hyper_graph.h"
#include "sparse_block_matrix.h"

namespace g2o {

class SparseOptimizer;

/**
 * \brief Generic interface for a sparse solver operating on a graph which
 * solves one iteration of the linearized objective function
 */
class G2O_CORE_API Solver {
 public:
  Solver();
  virtual ~Solver();
  Solver(const Solver&) = delete;
  Solver& operator=(const Solver&) = delete;

  /**
   * initialize the solver, called once before the first iteration
   */
  virtual bool init(SparseOptimizer* optimizer, bool online = false) = 0;

  /**
   * build the structure of the system
   */
  virtual bool buildStructure(bool zeroBlocks = false) = 0;
  /**
   * update the structures for online processing
   */
  virtual bool updateStructure(const HyperGraph::VertexContainer& vset,
                               const HyperGraph::EdgeSet& edges) = 0;
  /**
   * build the current system
   */
  virtual bool buildSystem() = 0;

  /**
   * solve Ax = b
   */
  virtual bool solve() = 0;

  /**
   * computes the block diagonal elements of the pattern specified in the input
   * and stores them in given SparseBlockMatrix
   */
  virtual bool computeMarginals(
      SparseBlockMatrix<MatrixX>& spinv,
      const std::vector<std::pair<int, int> >& blockIndices) = 0;

  /**
   * update the system while performing Levenberg, i.e., modifying the diagonal
   * components of A by doing += lambda along the main diagonal of the Matrix.
   * Note that this function may be called with a positive and a negative
   * lambda. The latter is used to undo a former modification. If backup is
   * true, then the solver should store a backup of the diagonal, which can be
   * restored by restoreDiagonal()
   */
  virtual bool setLambda(number_t lambda, bool backup = false) = 0;

  /**
   * restore a previously made backup of the diagonal
   */
  virtual void restoreDiagonal() = 0;

  //! return x, the solution vector
  number_t* x() { return x_; }
  const number_t* x() const { return x_; }
  //! return b, the right hand side of the system
  number_t* b() { return b_; }
  const number_t* b() const { return b_; }

  //! return the size of the solution vector (x) and b
  size_t vectorSize() const { return xSize_; }

  //! the optimizer (graph) on which the solver works
  SparseOptimizer* optimizer() const { return optimizer_; }
  void setOptimizer(SparseOptimizer* optimizer);

  //! the system is Levenberg-Marquardt
  bool levenberg() const { return isLevenberg_; }
  void setLevenberg(bool levenberg);

  /**
   * does this solver support the Schur complement for solving a system
   * consisting of poses and landmarks. Re-implement in a derived solver, if
   * your solver supports it.
   */
  virtual bool supportsSchur() { return false; }

  //! should the solver perform the schur complement or not
  virtual bool schur() = 0;
  virtual void setSchur(bool s) = 0;

  size_t additionalVectorSpace() const { return additionalVectorSpace_; }
  void setAdditionalVectorSpace(size_t s);

  /**
   * write debug output of the Hessian if system is not positive definite
   */
  virtual void setWriteDebug(bool) = 0;
  virtual bool writeDebug() const = 0;

  //! write the hessian to disk using the specified file name
  virtual bool saveHessian(const std::string& /*fileName*/) const = 0;

 protected:
  SparseOptimizer* optimizer_{nullptr};
  number_t* x_{nullptr};
  number_t* b_{nullptr};
  size_t xSize_{0}, maxXSize_{0};
  bool isLevenberg_{
      false};  ///< the system we gonna solve is a Levenberg-Marquardt system
  size_t additionalVectorSpace_{0};

  void resizeVector(size_t sx);
};
}  // namespace g2o

#endif
