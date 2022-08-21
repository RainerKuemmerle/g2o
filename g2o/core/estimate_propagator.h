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

#ifndef G2O_ESTIMATE_PROPAGATOR_H
#define G2O_ESTIMATE_PROPAGATOR_H

#include <limits>
#include <map>
#include <unordered_map>

#include "g2o_core_api.h"
#include "optimizable_graph.h"
#include "sparse_optimizer.h"

namespace g2o {

/**
 * \brief cost for traversing along active edges in the optimizer
 *
 * You may derive an own one, if necessary. The default is to return
 * initialEstimatePossible(from, to) for the edge.
 */
class G2O_CORE_API EstimatePropagatorCost {
 public:
  virtual ~EstimatePropagatorCost() = default;
  explicit EstimatePropagatorCost(SparseOptimizer* graph);
  virtual number_t operator()(OptimizableGraph::Edge* edge,
                              const OptimizableGraph::VertexSet& from,
                              OptimizableGraph::Vertex* to_) const;
  virtual const char* name() const { return "spanning tree"; }

 protected:
  SparseOptimizer* graph_;
};

/**
 * \brief cost for traversing only odometry edges.
 *
 * Initialize your graph along odometry edges. An odometry edge is assumed to
 * connect vertices whose IDs only differs by one.
 */
class G2O_CORE_API EstimatePropagatorCostOdometry
    : public EstimatePropagatorCost {
 public:
  explicit EstimatePropagatorCostOdometry(SparseOptimizer* graph);
  number_t operator()(OptimizableGraph::Edge* edge,
                      const OptimizableGraph::VertexSet& from_,
                      OptimizableGraph::Vertex* to_) const override;
  const char* name() const override { return "odometry"; }
};

/**
 * \brief propagation of an initial guess
 */
class G2O_CORE_API EstimatePropagator {
 public:
  virtual ~EstimatePropagator() = default;
  /**
   * \brief Applying the action for propagating.
   *
   * You may derive an own one, if necessary. The default is to call
   * initialEstimate(from, to) for the edge.
   */
  struct G2O_CORE_API PropagateAction {
    virtual ~PropagateAction() = default;
    virtual void operator()(OptimizableGraph::Edge* e,
                            const OptimizableGraph::VertexSet& from,
                            OptimizableGraph::Vertex* to) const {
      if (!to->fixed()) e->initialEstimate(from, to);
    }
  };

  using PropagateCost = EstimatePropagatorCost;

  class AdjacencyMapEntry;

  /**
   * \brief priority queue for AdjacencyMapEntry
   */
  class G2O_CORE_API PriorityQueue : public std::multimap<number_t, AdjacencyMapEntry*> {
   public:
    void push(AdjacencyMapEntry* entry);
    AdjacencyMapEntry* pop();
  };

  /**
   * \brief data structure for loopuk during Dijkstra
   */
  class G2O_CORE_API AdjacencyMapEntry {
   public:
    friend class EstimatePropagator;
    friend class PriorityQueue;
    AdjacencyMapEntry();
    void reset();
    std::shared_ptr<OptimizableGraph::Vertex> child() const { return child_; }
    const OptimizableGraph::VertexSet& parent() const { return parent_; }
    std::shared_ptr<OptimizableGraph::Edge> edge() const { return edge_; }
    number_t distance() const { return distance_; }
    int frontierLevel() const { return frontierLevel_; }

   protected:
    std::shared_ptr<OptimizableGraph::Vertex> child_;
    OptimizableGraph::VertexSet parent_;
    std::shared_ptr<OptimizableGraph::Edge> edge_;
    number_t distance_;
    int frontierLevel_;

   private:  // for PriorityQueue
    bool inQueue_;
    PriorityQueue::iterator queueIt_;
  };

  /**
   * \brief hash function for a vertex
   */
  class G2O_CORE_API VertexIDHashFunction {
   public:
    size_t operator()(
        const std::shared_ptr<OptimizableGraph::Vertex>& v) const {
      return v->id();
    }
  };

  using AdjacencyMap =
      std::unordered_map<std::shared_ptr<OptimizableGraph::Vertex>,
                         AdjacencyMapEntry, VertexIDHashFunction>;

  explicit EstimatePropagator(OptimizableGraph* g);
  OptimizableGraph::VertexSet& visited() { return visited_; }
  AdjacencyMap& adjacencyMap() { return adjacencyMap_; }
  OptimizableGraph* graph() { return graph_; }

  /**
   * propagate an initial guess starting from v. The function computes a
   * spanning tree whereas the cost for each edge is determined by calling
   * cost() and the action applied to each vertex is action().
   */
  void propagate(
      const std::shared_ptr<OptimizableGraph::Vertex>& v,
      const EstimatePropagator::PropagateCost& cost,
      const EstimatePropagator::PropagateAction& action = PropagateAction(),
      number_t maxDistance = std::numeric_limits<number_t>::max(),
      number_t maxEdgeCost = std::numeric_limits<number_t>::max());

  /**
   * same as above but starting to propagate from a set of vertices instead of
   * just a single one.
   */
  void propagate(
      OptimizableGraph::VertexSet& vset,
      const EstimatePropagator::PropagateCost& cost,
      const EstimatePropagator::PropagateAction& action = PropagateAction(),
      number_t maxDistance = std::numeric_limits<number_t>::max(),
      number_t maxEdgeCost = std::numeric_limits<number_t>::max());

 protected:
  void reset();

  AdjacencyMap adjacencyMap_;
  OptimizableGraph::VertexSet visited_;
  OptimizableGraph* graph_;
};

}  // namespace g2o
#endif
