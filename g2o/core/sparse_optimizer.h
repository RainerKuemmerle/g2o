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

#ifndef G2O_GRAPH_OPTIMIZER_CHOL_H_
#define G2O_GRAPH_OPTIMIZER_CHOL_H_

#include <memory>

#include "batch_stats.h"
#include "g2o/stuff/macros.h"
#include "g2o_core_api.h"
#include "optimizable_graph.h"
#include "sparse_block_matrix.h"

namespace g2o {

// forward declaration
class OptimizationAlgorithm;
class EstimatePropagatorCost;

class G2O_CORE_API SparseOptimizer : public OptimizableGraph {
 public:
  enum {
    kAtComputeactiverror = OptimizableGraph::kAtNumElements,
    kAtNumElements,  // keep as last element
  };

  friend class ActivePathCostFunction;

  // Attention: _solver & _statistics is own by SparseOptimizer and will be
  // deleted in its destructor.
  SparseOptimizer();
  ~SparseOptimizer() override;

  // new interface for the optimizer
  // the old functions will be dropped
  /**
   * Initializes the structures for optimizing a portion of the graph specified
   * by a subset of edges. Before calling it be sure to invoke marginalized()
   * and fixed() to the vertices you want to include in the schur complement or
   * to set as fixed during the optimization.
   * @param eset: the subgraph to be optimized.
   * @returns false if somethings goes wrong
   */
  virtual bool initializeOptimization(HyperGraph::EdgeSet& eset);

  /**
   * Initializes the structures for optimizing a portion of the graph specified
   * by a subset of vertices. Before calling it be sure to invoke marginalized()
   * and fixed() to the vertices you want to include in the schur complement or
   * to set as fixed during the optimization.
   * @param vset: the subgraph to be optimized.
   * @param level: is the level (in multilevel optimization)
   * @returns false if somethings goes wrong
   */
  virtual bool initializeOptimization(HyperGraph::VertexSet& vset,
                                      int level = 0);

  /**
   * Initializes the structures for optimizing the whole graph.
   * Before calling it be sure to invoke marginalized() and fixed() to the
   * vertices you want to include in the schur complement or to set as fixed
   * during the optimization.
   * @param level: is the level (in multilevel optimization)
   * @returns false if somethings goes wrong
   */
  virtual bool initializeOptimization(int level = 0);

  /**
   * HACK updating the internal structures for online processing
   */
  virtual bool updateInitialization(HyperGraph::VertexSet& vset,
                                    HyperGraph::EdgeSet& eset);

  /**
   * Propagates an initial guess from the vertex specified as origin.
   * It should be called after initializeOptimization(...), as it relies on the
   * _activeVertices/_edges structures. It constructs a set of trees starting
   * from the nodes in the graph which are fixed and eligible for preforming
   * optimization. The trees are constructed by utlizing a cost-function
   * specified.
   * @param costFunction: the cost function used
   * @patam maxDistance: the distance where to stop the search
   */
  virtual void computeInitialGuess();

  /**
   * Same as above but using a specific propagator
   */
  virtual void computeInitialGuess(EstimatePropagatorCost& propagator);

  /**
   * sets all vertices to their origin.
   */
  virtual void setToOrigin();

  /**
   * starts one optimization run given the current configuration of the graph,
   * and the current settings stored in the class instance.
   * It can be called only after initializeOptimization
   */
  int optimize(int iterations, bool online = false) override;

  /**
   * computes the blocks of the inverse of the specified pattern.
   * the pattern is given via pairs <row, col> of the blocks in the hessian
   * @param blockIndices: the pattern
   * @param spinv: the sparse block matrix with the result
   * @returns false if the operation is not supported by the solver
   */
  bool computeMarginals(SparseBlockMatrix<MatrixX>& spinv,
                        const std::vector<std::pair<int, int> >& blockIndices);

  /**
   * computes the inverse of the specified vertex.
   * @param vertex: the vertex whose state is to be marginalised
   * @param spinv: the sparse block matrix with the result
   * @returns false if the operation is not supported by the solver
   */
  bool computeMarginals(SparseBlockMatrix<MatrixX>& spinv,
                        const Vertex* vertex) {
    if (vertex->hessianIndex() < 0) {
      return false;
    }
    std::vector<std::pair<int, int> > index;
    index.emplace_back(vertex->hessianIndex(), vertex->hessianIndex());
    return computeMarginals(spinv, index);
  }

  /**
   * computes the inverse of the set specified vertices, assembled into a single
   * covariance matrix.
   * @param vertex: the pattern
   * @param spinv: the sparse block matrix with the result
   * @returns false if the operation is not supported by the solver
   */
  bool computeMarginals(SparseBlockMatrix<MatrixX>& spinv,
                        const VertexContainer& vertices) {
    std::vector<std::pair<int, int> > indices;
    for (const auto& vertex : vertices) {
      indices.emplace_back(vertex->hessianIndex(), vertex->hessianIndex());
    }
    return computeMarginals(spinv, indices);
  }

  //! finds a gauge in the graph to remove the undefined dof.
  // The gauge should be fixed() and then the optimization can work (if no
  // additional dof are in the system. The default implementation returns a node
  // with maximum dimension.
  virtual std::shared_ptr<OptimizableGraph::Vertex> findGauge();

  bool gaugeFreedom();

  /**returns the cached chi2 of the active portion of the graph*/
  number_t activeChi2() const;
  /**
   * returns the cached chi2 of the active portion of the graph.
   * In contrast to activeChi2() this functions considers the weighting
   * of the error according to the robustification of the error functions.
   */
  number_t activeRobustChi2() const;

  //! verbose information during optimization
  bool verbose() const { return verbose_; }
  void setVerbose(bool verbose);

  /**
   * sets a variable checked at every iteration to force a user stop. The
   * iteration exits when the variable is true;
   */
  void setForceStopFlag(bool* flag);
  bool* forceStopFlag() const { return forceStopFlag_; };

  //! if external stop flag is given, return its state. False otherwise
  bool terminate() const { return forceStopFlag_ ? (*forceStopFlag_) : false; }

  //! the index mapping of the vertices
  const VertexContainerRaw& indexMapping() const { return ivMap_; }
  //! the vertices active in the current optimization
  const VertexContainer& activeVertices() const { return activeVertices_; }
  //! the edges active in the current optimization
  const EdgeContainer& activeEdges() const { return activeEdges_; }

  /**
   * Remove a vertex. If the vertex is contained in the currently active set
   * of vertices, then the internal temporary structures are cleaned, e.g., the
   * index mapping is erased. In case you need the index mapping for
   * manipulating the graph, you have to store it in your own copy.
   */
  bool removeVertex(const std::shared_ptr<HyperGraph::Vertex>& v,
                    bool detach = false) override;

  /**
   * search for an edge in _activeVertices and return the iterator pointing to
   * it getActiveVertices().end() if not found
   */
  VertexContainer::const_iterator findActiveVertex(
      const OptimizableGraph::Vertex* v) const;
  /**
   * search for an edge in _activeEdges and return the iterator pointing to it
   * getActiveEdges().end() if not found
   */
  EdgeContainer::const_iterator findActiveEdge(
      const OptimizableGraph::Edge* e) const;

  //! the solver used by the optimizer
  std::shared_ptr<OptimizationAlgorithm> algorithm() const {
    return algorithm_;
  }
  std::shared_ptr<OptimizationAlgorithm> solver() { return algorithm_; }
  void setAlgorithm(const std::shared_ptr<OptimizationAlgorithm>& algorithm);

  //! push the estimate of a subset of the variables onto a stack
  virtual void push(SparseOptimizer::VertexContainer& vlist);
  //! push the estimate of a subset of the variables onto a stack
  void push(HyperGraph::VertexSet& vlist) override;
  //! push all the active vertices onto a stack
  void push() override;
  //! pop (restore) the estimate a subset of the variables from the stack
  virtual void pop(SparseOptimizer::VertexContainer& vlist);
  //! pop (restore) the estimate a subset of the variables from the stack
  void pop(HyperGraph::VertexSet& vlist) override;
  //! pop (restore) the estimate of the active vertices from the stack
  void pop() override;

  //! ignore the latest stored element on the stack, remove it from the stack
  //! but do not restore the estimate
  virtual void discardTop(SparseOptimizer::VertexContainer& vlist);
  //! same as above, but for the active vertices
  void discardTop() override;
  using OptimizableGraph::discardTop;

  /**
   * clears the graph, and polishes some intermediate structures
   * Note that this only removes nodes / edges. Parameters can be removed
   * with clearParameters().
   */
  void clear() override;

  /**
   * computes the error vectors of all edges in the activeSet, and caches them
   */
  void computeActiveErrors();

  /**
   * Linearizes the system by computing the Jacobians for the nodes
   * and edges in the graph
   */
  G2O_ATTRIBUTE_DEPRECATED(void linearizeSystem()) {
    // nothing needed, linearization is now done inside the solver
  }

  /**
   * update the estimate of the active vertices
   * @param update: the number_t vector containing the stacked
   * elements of the increments on the vertices.
   */
  void update(number_t* update);

  /**
     returns the set of batch statistics about the optimisation
  */
  const BatchStatisticsContainer& batchStatistics() const {
    return batchStatistics_;
  }
  /**
     returns the set of batch statistics about the optimisation
  */
  BatchStatisticsContainer& batchStatistics() { return batchStatistics_; }

  void setComputeBatchStatistics(bool computeBatchStatistics);

  bool computeBatchStatistics() const { return computeBatchStatistics_; }

  /**** callbacks ****/
  //! add an action to be executed before the error vectors are computed
  bool addComputeErrorAction(const std::shared_ptr<HyperGraphAction>& action);
  //! remove an action that should no longer be executed before computing the
  //! error vectors
  bool removeComputeErrorAction(
      const std::shared_ptr<HyperGraphAction>& action);

 protected:
  bool* forceStopFlag_{nullptr};
  bool verbose_{false};

  VertexContainerRaw ivMap_;
  VertexContainer activeVertices_;  ///< sorted according to VertexIDCompare
  EdgeContainer activeEdges_;       ///< sorted according to EdgeIDCompare

  void sortVectorContainers();

  std::shared_ptr<OptimizationAlgorithm> algorithm_;

  /**
   * builds the mapping of the active vertices to the (block) row / column in
   * the Hessian
   */
  bool buildIndexMapping(SparseOptimizer::VertexContainer& vlist);
  void clearIndexMapping();

  BatchStatisticsContainer
      batchStatistics_;  ///< global statistics of the optimizer, e.g., timing,
                         ///< num-non-zeros
  bool computeBatchStatistics_{false};
};
}  // namespace g2o

#endif
