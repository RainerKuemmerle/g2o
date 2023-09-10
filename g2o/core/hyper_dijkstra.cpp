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

#include "hyper_dijkstra.h"

#include <cassert>
#include <cstddef>
#include <deque>
#include <queue>
#include <unordered_map>
#include <utility>

#include "g2o/core/hyper_graph.h"
#include "g2o/stuff/logger.h"

namespace g2o {

double HyperDijkstra::TreeAction::perform(
    const std::shared_ptr<HyperGraph::Vertex>& v,
    const std::shared_ptr<HyperGraph::Vertex>& vParent,
    const std::shared_ptr<HyperGraph::Edge>& e) {
  (void)v;
  (void)vParent;
  (void)e;
  return std::numeric_limits<double>::max();
}

double HyperDijkstra::TreeAction::perform(
    const std::shared_ptr<HyperGraph::Vertex>& v,
    const std::shared_ptr<HyperGraph::Vertex>& vParent,
    const std::shared_ptr<HyperGraph::Edge>& e, double distance) {
  if (distance == -1) return perform(v, vParent, e);
  return std::numeric_limits<double>::max();
}

HyperDijkstra::AdjacencyMapEntry::AdjacencyMapEntry(
    std::shared_ptr<HyperGraph::Vertex> child,
    std::shared_ptr<HyperGraph::Vertex> parent,
    std::shared_ptr<HyperGraph::Edge> edge, double distance)
    : child_(std::move(child)),
      parent_(std::move(parent)),
      edge_(std::move(edge)),
      distance_(distance) {}

HyperDijkstra::HyperDijkstra(std::shared_ptr<HyperGraph> g)
    : graph_(std::move(g)) {
  for (const auto& it : graph_->vertices()) {
    const AdjacencyMapEntry entry(it.second, nullptr, nullptr,
                                  std::numeric_limits<double>::max());
    adjacencyMap_.emplace(it.second, entry);
  }
}

void HyperDijkstra::reset() {
  for (const auto& it : visited_) {
    auto at = adjacencyMap_.find(it);
    assert(at != adjacencyMap_.end());
    at->second = AdjacencyMapEntry(at->first, nullptr, nullptr,
                                   std::numeric_limits<double>::max());
  }
  visited_.clear();
}

bool operator<(const HyperDijkstra::AdjacencyMapEntry& a,
               const HyperDijkstra::AdjacencyMapEntry& b) {
  return a.distance() > b.distance();
}

void HyperDijkstra::shortestPaths(HyperGraph::VertexSet& vset,
                                  HyperDijkstra::CostFunction& cost,
                                  double maxDistance,
                                  double comparisonConditioner, bool directed,
                                  double maxEdgeCost) {
  reset();
  std::priority_queue<AdjacencyMapEntry> frontier;
  for (const auto& v : vset) {
    assert(v != nullptr);
    auto it = adjacencyMap_.find(v);
    if (it == adjacencyMap_.end()) {
      G2O_WARN("{} Vertex {} is not in the adjacency map", __PRETTY_FUNCTION__,
               v->id());
    }
    assert(it != adjacencyMap_.end());
    it->second.distance_ = 0.;
    it->second.parent_ = nullptr;
    frontier.push(it->second);
  }

  while (!frontier.empty()) {
    const AdjacencyMapEntry entry = frontier.top();
    frontier.pop();
    auto u = entry.child();
    auto ut = adjacencyMap_.find(u);
    if (ut == adjacencyMap_.end()) {
      G2O_WARN("{} Vertex {} is not in the adjacency map", __PRETTY_FUNCTION__,
               u->id());
    }
    assert(ut != adjacencyMap_.end());
    const double uDistance = ut->second.distance();

    const std::pair<HyperGraph::VertexSet::iterator, bool> insertResult =
        visited_.insert(u);
    (void)insertResult;
    auto et = u->edges().begin();
    while (et != u->edges().end()) {
      auto edge = et->lock();
      ++et;

      if (directed && edge->vertex(0) != u) continue;

      for (size_t i = 0; i < edge->vertices().size(); ++i) {
        auto z = edge->vertex(i);
        if (z == u) continue;

        const double edgeDistance = cost(edge.get(), u.get(), z.get());
        if (edgeDistance == std::numeric_limits<double>::max() ||
            edgeDistance > maxEdgeCost)
          continue;
        const double zDistance = uDistance + edgeDistance;

        auto ot = adjacencyMap_.find(z);
        assert(ot != adjacencyMap_.end());

        if (zDistance + comparisonConditioner < ot->second.distance() &&
            zDistance < maxDistance) {
          ot->second.distance_ = zDistance;
          ot->second.parent_ = u;
          ot->second.edge_ = edge;
          frontier.push(ot->second);
        }
      }
    }
  }
}

void HyperDijkstra::shortestPaths(const std::shared_ptr<HyperGraph::Vertex>& v,
                                  HyperDijkstra::CostFunction& cost,
                                  double maxDistance,
                                  double comparisonConditioner, bool directed,
                                  double maxEdgeCost) {
  HyperGraph::VertexSet vset;
  vset.insert(v);
  shortestPaths(vset, cost, maxDistance, comparisonConditioner, directed,
                maxEdgeCost);
}

void HyperDijkstra::computeTree(AdjacencyMap& amap) {
  for (auto& it : amap) {
    it.second.children_.clear();
  }
  for (auto it = amap.begin(); it != amap.end(); ++it) {
    const AdjacencyMapEntry& entry(it->second);
    auto parent = entry.parent();
    if (!parent) {
      continue;
    }
    auto v = entry.child();
    assert(v == it->first);

    auto pt = amap.find(parent);
    assert(pt != amap.end());
    pt->second.children_.insert(v);
  }
}

void HyperDijkstra::visitAdjacencyMap(AdjacencyMap& amap, TreeAction& action,
                                      bool useDistance) {
  using Deque = std::deque<std::shared_ptr<HyperGraph::Vertex>>;
  Deque q;
  // scans for the vertices without the parent (which are the roots of the
  // trees) and applies the action to them.
  for (auto& it : amap) {
    const AdjacencyMapEntry& entry(it.second);
    if (!entry.parent()) {
      action.perform(it.first, nullptr, nullptr);
      q.push_back(it.first);
    }
  }

  while (!q.empty()) {
    auto parent = q.front();
    q.pop_front();
    auto parentIt = amap.find(parent);
    if (parentIt == amap.end()) {
      continue;
    }
    const HyperGraph::VertexSet& childs = parentIt->second.children();
    for (const auto& child : childs) {
      auto adjacencyIt = amap.find(child);
      assert(adjacencyIt != amap.end());
      auto edge = adjacencyIt->second.edge();

      assert(adjacencyIt->first == child);
      assert(adjacencyIt->second.child() == child);
      assert(adjacencyIt->second.parent() == parent);
      if (!useDistance) {
        action.perform(child, parent, edge);
      } else {
        action.perform(child, parent, edge, adjacencyIt->second.distance());
      }
      q.push_back(child);
    }
  }
}

double UniformCostFunction::operator()(HyperGraph::Edge* /*edge*/,
                                       HyperGraph::Vertex* /*from*/,
                                       HyperGraph::Vertex* /*to*/) {
  return 1.;
}

};  // namespace g2o
