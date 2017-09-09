// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#ifndef G2O_EDGE_LABELER_
#define G2O_EDGE_LABELER_

#include "g2o/core/sparse_optimizer.h"
#include <Eigen/Core>

#include "g2o_hierarchical_api.h"

namespace g2o {
  /**
   * This class implements the functions to label an edge (measurement) based
   * on the actual configuration of the nodes. It does so by
   * <ul>
   * <li> computing the expected mean of the measurement (_measurement) based on the state variables
   * <li> computing the joint covariance matrix of all sstate variables on which the measurement depends.
   * <li> extracting the sigma points from this covariance matrix (which is in the space if the increments used by \oplus
   * <li> projecting the sigma points in the error space, and thus computing the information matrix of the labeled edge
   * </ul>
   */
struct G2O_HIERARCHICAL_API EdgeLabeler{
  //! constructs an edge labeler that operates on the optimizer passed as argument
  //! @param optimizer: the optimizer
  EdgeLabeler(SparseOptimizer* optimizer);

  //! Labels the set of edges passed as argument. It computes the cholesky information matrix.
  //! This method only woorks aftec having called an optimize(...) in the connected optimizer.
  //! The labeling is performed based on the actual configuration of the nodes in the optimized subgraph.
  //! @param edges: the edges to label
  //! @returns -1 if the inverse cholesky cannot be computed, otherwise the number of edges where the labeling was successful
  int labelEdges(std::set<OptimizableGraph::Edge*>& edges);

protected:
  //! helper function that augments the sparse pattern of the inverse based on an edge
  //! @param pattern: the blocks of the inverse covered by the edge
  //! @param e: the edge
  void augmentSparsePattern(std::set<std::pair<int, int> >& pattern, OptimizableGraph::Edge* e);

  //! helper function that computes the inverse based on the sparse pattenrn
  //! @param spinv:   the output block inverse
  //! @param pattern: the blocks of the inverse covered by the edge
  //! @returns true on successm, false on failure, .
  bool computePartialInverse(SparseBlockMatrix<Eigen::MatrixXd>& spinv, const std::set<std::pair<int,int> >& pattern);

  //! helper function that labes a specific edge based on the marginals in the sparse block inverse
  //! @returns true on success, false on failure
  bool labelEdge( const SparseBlockMatrix<Eigen::MatrixXd>& spinv, OptimizableGraph::Edge* e);
   SparseOptimizer* _optimizer;
};

} // end namespace
#endif
