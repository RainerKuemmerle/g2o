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
