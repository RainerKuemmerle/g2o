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

#include "estimate_propagator.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>

//#define DEBUG_ESTIMATE_PROPAGATOR

namespace g2o {

#ifdef DEBUG_ESTIMATE_PROPAGATOR
struct FrontierLevelCmp {
  bool operator()(EstimatePropagator::AdjacencyMapEntry* e1,
                  EstimatePropagator::AdjacencyMapEntry* e2) const {
    return e1->frontierLevel() < e2->frontierLevel();
  }
};
#endif

EstimatePropagator::AdjacencyMapEntry::AdjacencyMapEntry() { reset(); }

void EstimatePropagator::AdjacencyMapEntry::reset() {
  child_ = nullptr;
  parent_.clear();
  edge_ = nullptr;
  distance_ = std::numeric_limits<number_t>::max();
  frontierLevel_ = -1;
  inQueue_ = false;
}

EstimatePropagator::EstimatePropagator(OptimizableGraph* g) : graph_(g) {
  for (const auto& it : graph_->vertices()) {
    AdjacencyMapEntry entry;
    entry.child_ =
        std::static_pointer_cast<OptimizableGraph::Vertex>(it.second);
    adjacencyMap_.insert(make_pair(entry.child(), entry));
  }
}

void EstimatePropagator::reset() {
  for (const auto& it : visited_) {
    auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(it);
    auto at = adjacencyMap_.find(v);
    assert(at != adjacencyMap_.end());
    at->second.reset();
  }
  visited_.clear();
}

void EstimatePropagator::propagate(
    const std::shared_ptr<OptimizableGraph::Vertex>& v,
    const EstimatePropagator::PropagateCost& cost,
    const EstimatePropagator::PropagateAction& action, number_t maxDistance,
    number_t maxEdgeCost) {
  OptimizableGraph::VertexSet vset;
  vset.insert(v);
  propagate(vset, cost, action, maxDistance, maxEdgeCost);
}

void EstimatePropagator::propagate(
    OptimizableGraph::VertexSet& vset,
    const EstimatePropagator::PropagateCost& cost,
    const EstimatePropagator::PropagateAction& action, number_t maxDistance,
    number_t maxEdgeCost) {
  reset();

  PriorityQueue frontier;
  for (const auto& vit : vset) {
    auto v = std::static_pointer_cast<OptimizableGraph::Vertex>(vit);
    auto it = adjacencyMap_.find(v);
    assert(it != adjacencyMap_.end());
    it->second.distance_ = 0.;
    it->second.parent_.clear();
    it->second.frontierLevel_ = 0;
    frontier.push(&it->second);
  }

  while (!frontier.empty()) {
    AdjacencyMapEntry* entry = frontier.pop();
    const auto& u = entry->child();
    const number_t uDistance = entry->distance();
    // cerr << "uDistance " << uDistance << endl;

    // initialize the vertex
    if (entry->frontierLevel_ > 0) {
      action(entry->edge().get(), entry->parent(), u.get());
    }

    /* std::pair< OptimizableGraph::VertexSet::iterator, bool> insertResult = */
    visited_.insert(u);
    auto et = u->edges().begin();
    while (et != u->edges().end()) {
      auto edge = std::static_pointer_cast<OptimizableGraph::Edge>(et->lock());
      ++et;

      int maxFrontier = -1;
      OptimizableGraph::VertexSet initializedVertices;
      for (size_t i = 0; i < edge->vertices().size(); ++i) {
        auto z =
            std::static_pointer_cast<OptimizableGraph::Vertex>(edge->vertex(i));
        if (!z) continue;
        auto ot = adjacencyMap_.find(z);
        if (ot->second.distance_ != std::numeric_limits<number_t>::max()) {
          initializedVertices.insert(z);
          maxFrontier = (std::max)(maxFrontier, ot->second.frontierLevel_);
        }
      }
      assert(maxFrontier >= 0);

      for (size_t i = 0; i < edge->vertices().size(); ++i) {
        auto z =
            std::static_pointer_cast<OptimizableGraph::Vertex>(edge->vertex(i));
        if (!z) continue;
        if (z == u) continue;
        const size_t wasInitialized = initializedVertices.erase(z);

        const number_t edgeDistance =
            cost(edge.get(), initializedVertices, z.get());
        if (edgeDistance > 0. &&
            edgeDistance != std::numeric_limits<number_t>::max() &&
            edgeDistance < maxEdgeCost) {
          const number_t zDistance = uDistance + edgeDistance;
          // cerr << z->id() << " " << zDistance << endl;

          auto ot = adjacencyMap_.find(z);
          assert(ot != adjacencyMap_.end());

          if (zDistance < ot->second.distance() && zDistance < maxDistance) {
            // if (ot->second.inQueue)
            // cerr << "Updating" << endl;
            ot->second.distance_ = zDistance;
            ot->second.parent_ = initializedVertices;
            ot->second.edge_ = edge;
            ot->second.frontierLevel_ = maxFrontier + 1;
            frontier.push(&ot->second);
          }
        }

        if (wasInitialized > 0) initializedVertices.insert(z);
      }
    }
  }

  // writing debug information like cost for reaching each vertex and the parent
  // used to initialize
#ifdef DEBUG_ESTIMATE_PROPAGATOR
  cerr << "Writing cost.dat" << endl;
  ofstream costStream("cost.dat");
  for (AdjacencyMap::const_iterator it = adjacencyMap_.begin();
       it != adjacencyMap_.end(); ++it) {
    HyperGraph::Vertex* u = it->second.child();
    costStream << "vertex " << u->id() << "  cost " << it->second._distance
               << endl;
  }
  cerr << "Writing init.dat" << endl;
  ofstream initStream("init.dat");
  vector<AdjacencyMapEntry*> frontierLevels;
  for (AdjacencyMap::iterator it = adjacencyMap_.begin();
       it != adjacencyMap_.end(); ++it) {
    if (it->second._frontierLevel > 0) frontierLevels.push_back(&it->second);
  }
  sort(frontierLevels.begin(), frontierLevels.end(), FrontierLevelCmp());
  for (vector<AdjacencyMapEntry*>::const_iterator it = frontierLevels.begin();
       it != frontierLevels.end(); ++it) {
    AdjacencyMapEntry* entry = *it;
    OptimizableGraph::Vertex* to = entry->child();

    initStream << "calling init level = " << entry->_frontierLevel << "\t (";
    for (OptimizableGraph::VertexSet::iterator pit = entry->parent().begin();
         pit != entry->parent().end(); ++pit) {
      initStream << " " << (*pit)->id();
    }
    initStream << " ) -> " << to->id() << endl;
  }
#endif
}

void EstimatePropagator::PriorityQueue::push(AdjacencyMapEntry* entry) {
  assert(entry != nullptr);
  if (entry->inQueue_) {
    assert(entry->queueIt_->second == entry);
    erase(entry->queueIt_);
  }

  entry->queueIt_ = insert(std::make_pair(entry->distance(), entry));
  assert(entry->queueIt_ != end());
  entry->inQueue_ = true;
}

EstimatePropagator::AdjacencyMapEntry*
EstimatePropagator::PriorityQueue::pop() {
  assert(!empty());
  auto it = begin();
  AdjacencyMapEntry* entry = it->second;
  erase(it);

  assert(entry != nullptr);
  entry->queueIt_ = end();
  entry->inQueue_ = false;
  return entry;
}

EstimatePropagatorCost::EstimatePropagatorCost(SparseOptimizer* graph)
    : graph_(graph) {}

number_t EstimatePropagatorCost::operator()(
    OptimizableGraph::Edge* edge, const OptimizableGraph::VertexSet& from,
    OptimizableGraph::Vertex* to_) const {
  auto* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
  auto* to = dynamic_cast<OptimizableGraph::Vertex*>(to_);
  auto it = graph_->findActiveEdge(e);
  if (it == graph_->activeEdges().end())  // it has to be an active edge
    return std::numeric_limits<number_t>::max();
  return e->initialEstimatePossible(from, to);
}

EstimatePropagatorCostOdometry::EstimatePropagatorCostOdometry(
    SparseOptimizer* graph)
    : EstimatePropagatorCost(graph) {}

number_t EstimatePropagatorCostOdometry::operator()(
    OptimizableGraph::Edge* edge, const OptimizableGraph::VertexSet& from_,
    OptimizableGraph::Vertex* to_) const {
  auto* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
  OptimizableGraph::Vertex* from =
      dynamic_cast<OptimizableGraph::Vertex*>(from_.begin()->get());
  auto* to = dynamic_cast<OptimizableGraph::Vertex*>(to_);
  if (std::abs(from->id() - to->id()) !=
      1)  // simple method to identify odometry edges in a pose graph
    return std::numeric_limits<number_t>::max();
  auto it = graph_->findActiveEdge(e);
  if (it == graph_->activeEdges().end())  // it has to be an active edge
    return std::numeric_limits<number_t>::max();
  return e->initialEstimatePossible(from_, to);
}

}  // namespace g2o
