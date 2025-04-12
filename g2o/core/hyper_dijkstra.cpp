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
#include <deque>
#include <queue>

#include "g2o/stuff/logger.h"

namespace g2o {

using namespace std;

double HyperDijkstra::TreeAction::perform(HyperGraph::Vertex* v,
                                          HyperGraph::Vertex* vParent,
                                          HyperGraph::Edge* e) {
  (void)v;
  (void)vParent;
  (void)e;
  return std::numeric_limits<double>::max();
}

double HyperDijkstra::TreeAction::perform(HyperGraph::Vertex* v,
                                          HyperGraph::Vertex* vParent,
                                          HyperGraph::Edge* e,
                                          double distance) {
  if (distance == -1) return perform(v, vParent, e);
  return std::numeric_limits<double>::max();
}

HyperDijkstra::AdjacencyMapEntry::AdjacencyMapEntry(HyperGraph::Vertex* child_,
                                                    HyperGraph::Vertex* parent_,
                                                    HyperGraph::Edge* edge_,
                                                    double distance_)
    : _child(child_), _parent(parent_), _edge(edge_), _distance(distance_) {}

HyperDijkstra::HyperDijkstra(HyperGraph* g) : _graph(g) {
  for (HyperGraph::VertexIDMap::const_iterator it = _graph->vertices().begin();
       it != _graph->vertices().end(); ++it) {
    AdjacencyMapEntry entry(it->second, 0, 0,
                            std::numeric_limits<double>::max());
    _adjacencyMap.insert(make_pair(entry.child(), entry));
  }
}

void HyperDijkstra::reset() {
  for (HyperGraph::VertexSet::iterator it = _visited.begin();
       it != _visited.end(); ++it) {
    AdjacencyMap::iterator at = _adjacencyMap.find(*it);
    assert(at != _adjacencyMap.end());
    at->second =
        AdjacencyMapEntry(at->first, 0, 0, std::numeric_limits<double>::max());
  }
  _visited.clear();
}

bool operator<(const HyperDijkstra::AdjacencyMapEntry& a,
               const HyperDijkstra::AdjacencyMapEntry& b) {
  return a.distance() > b.distance();
}

void HyperDijkstra::shortestPaths(HyperGraph::VertexSet& vset,
                                  HyperDijkstra::CostFunction* cost,
                                  double maxDistance,
                                  double comparisonConditioner, bool directed,
                                  double maxEdgeCost) {
  reset();
  std::priority_queue<AdjacencyMapEntry> frontier;
  for (HyperGraph::VertexSet::iterator vit = vset.begin(); vit != vset.end();
       ++vit) {
    HyperGraph::Vertex* v = *vit;
    assert(v != 0);
    AdjacencyMap::iterator it = _adjacencyMap.find(v);
    if (it == _adjacencyMap.end()) {
      G2O_WARN("Vertex {} is not in the adjacency map", v->id());
    }
    assert(it != _adjacencyMap.end());
    it->second._distance = 0.;
    it->second._parent = 0;
    frontier.push(it->second);
  }

  while (!frontier.empty()) {
    AdjacencyMapEntry entry = frontier.top();
    frontier.pop();
    HyperGraph::Vertex* u = entry.child();
    AdjacencyMap::iterator ut = _adjacencyMap.find(u);
    if (ut == _adjacencyMap.end()) {
      G2O_WARN("Vertex {} is not in the adjacency map", u->id());
    }
    assert(ut != _adjacencyMap.end());
    double uDistance = ut->second.distance();

    std::pair<HyperGraph::VertexSet::iterator, bool> insertResult =
        _visited.insert(u);
    (void)insertResult;
    HyperGraph::EdgeSet::iterator et = u->edges().begin();
    while (et != u->edges().end()) {
      HyperGraph::Edge* edge = *et;
      ++et;

      if (directed && edge->vertex(0) != u) continue;

      for (size_t i = 0; i < edge->vertices().size(); ++i) {
        HyperGraph::Vertex* z = edge->vertex(i);
        if (z == u) continue;

        double edgeDistance = (*cost)(edge, u, z);
        if (edgeDistance == std::numeric_limits<double>::max() ||
            edgeDistance > maxEdgeCost)
          continue;
        double zDistance = uDistance + edgeDistance;

        AdjacencyMap::iterator ot = _adjacencyMap.find(z);
        assert(ot != _adjacencyMap.end());

        if (zDistance + comparisonConditioner < ot->second.distance() &&
            zDistance < maxDistance) {
          ot->second._distance = zDistance;
          ot->second._parent = u;
          ot->second._edge = edge;
          frontier.push(ot->second);
        }
      }
    }
  }
}

void HyperDijkstra::shortestPaths(HyperGraph::Vertex* v,
                                  HyperDijkstra::CostFunction* cost,
                                  double maxDistance,
                                  double comparisonConditioner, bool directed,
                                  double maxEdgeCost) {
  HyperGraph::VertexSet vset;
  vset.insert(v);
  shortestPaths(vset, cost, maxDistance, comparisonConditioner, directed,
                maxEdgeCost);
}

void HyperDijkstra::computeTree(AdjacencyMap& amap) {
  for (AdjacencyMap::iterator it = amap.begin(); it != amap.end(); ++it) {
    AdjacencyMapEntry& entry(it->second);
    entry._children.clear();
  }
  for (AdjacencyMap::iterator it = amap.begin(); it != amap.end(); ++it) {
    AdjacencyMapEntry& entry(it->second);
    HyperGraph::Vertex* parent = entry.parent();
    if (!parent) {
      continue;
    }
    HyperGraph::Vertex* v = entry.child();
    assert(v == it->first);

    AdjacencyMap::iterator pt = amap.find(parent);
    assert(pt != amap.end());
    pt->second._children.insert(v);
  }
}

void HyperDijkstra::visitAdjacencyMap(AdjacencyMap& amap, TreeAction* action,
                                      bool useDistance) {
  typedef std::deque<HyperGraph::Vertex*> Deque;
  Deque q;
  // scans for the vertices without the parent (which are the roots of the
  // trees) and applies the action to them.
  for (AdjacencyMap::iterator it = amap.begin(); it != amap.end(); ++it) {
    AdjacencyMapEntry& entry(it->second);
    if (!entry.parent()) {
      action->perform(it->first, 0, 0);
      q.push_back(it->first);
    }
  }

  while (!q.empty()) {
    HyperGraph::Vertex* parent = q.front();
    q.pop_front();
    AdjacencyMap::iterator parentIt = amap.find(parent);
    if (parentIt == amap.end()) {
      continue;
    }
    HyperGraph::VertexSet& childs(parentIt->second.children());
    for (HyperGraph::VertexSet::iterator childsIt = childs.begin();
         childsIt != childs.end(); ++childsIt) {
      HyperGraph::Vertex* child = *childsIt;
      AdjacencyMap::iterator adjacencyIt = amap.find(child);
      assert(adjacencyIt != amap.end());
      HyperGraph::Edge* edge = adjacencyIt->second.edge();

      assert(adjacencyIt->first == child);
      assert(adjacencyIt->second.child() == child);
      assert(adjacencyIt->second.parent() == parent);
      if (!useDistance) {
        action->perform(child, parent, edge);
      } else {
        action->perform(child, parent, edge, adjacencyIt->second.distance());
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
