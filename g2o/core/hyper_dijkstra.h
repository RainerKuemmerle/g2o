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

#ifndef G2O_AIS_GENERAL_DIJKSTRA_HH
#define G2O_AIS_GENERAL_DIJKSTRA_HH

#include <limits>
#include <map>
#include <set>

#include "hyper_graph.h"

namespace g2o {

struct G2O_CORE_API HyperDijkstra {
  struct G2O_CORE_API CostFunction {
    virtual number_t operator()(HyperGraph::Edge* e, HyperGraph::Vertex* from,
                                HyperGraph::Vertex* to) = 0;
    virtual ~CostFunction() {}
  };

  struct G2O_CORE_API TreeAction {
    virtual number_t perform(const std::shared_ptr<HyperGraph::Vertex>& v,
                             const std::shared_ptr<HyperGraph::Vertex>& vParent,
                             const std::shared_ptr<HyperGraph::Edge>& e);
    virtual number_t perform(const std::shared_ptr<HyperGraph::Vertex>& v,
                             const std::shared_ptr<HyperGraph::Vertex>& vParent,
                             const std::shared_ptr<HyperGraph::Edge>& e,
                             number_t distance);
  };

  struct G2O_CORE_API AdjacencyMapEntry {
    friend struct HyperDijkstra;
    AdjacencyMapEntry(const std::shared_ptr<HyperGraph::Vertex>& child = nullptr,
                      const std::shared_ptr<HyperGraph::Vertex>& parent = nullptr,
                      const std::shared_ptr<HyperGraph::Edge>& edge = nullptr,
                      number_t distance = std::numeric_limits<number_t>::max());
    std::shared_ptr<HyperGraph::Vertex> child() const { return _child; }
    std::shared_ptr<HyperGraph::Vertex> parent() const { return _parent; }
    std::shared_ptr<HyperGraph::Edge> edge() const { return _edge; }
    number_t distance() const { return _distance; }
    HyperGraph::VertexSet& children() { return _children; }
    const HyperGraph::VertexSet& children() const { return _children; }

   protected:
    std::shared_ptr<HyperGraph::Vertex> _child;
    std::shared_ptr<HyperGraph::Vertex> _parent;
    std::shared_ptr<HyperGraph::Edge> _edge;
    number_t _distance;
    HyperGraph::VertexSet _children;
  };

  using AdjacencyMap = std::map<std::shared_ptr<HyperGraph::Vertex>, AdjacencyMapEntry>;

  explicit HyperDijkstra(const std::shared_ptr<HyperGraph>& g);
  HyperGraph::VertexSet& visited() { return _visited; }
  const HyperGraph::VertexSet& visited() const { return _visited; }
  AdjacencyMap& adjacencyMap() { return _adjacencyMap; }
  const AdjacencyMap& adjacencyMap() const { return _adjacencyMap; }
  std::shared_ptr<HyperGraph> graph() const { return _graph; }

  void shortestPaths(const std::shared_ptr<HyperGraph::Vertex>& v,
                     HyperDijkstra::CostFunction* cost,
                     number_t maxDistance = std::numeric_limits<number_t>::max(),
                     number_t comparisonConditioner = 1e-3, bool directed = false,
                     number_t maxEdgeCost = std::numeric_limits<number_t>::max());

  void shortestPaths(HyperGraph::VertexSet& vset, HyperDijkstra::CostFunction* cost,
                     number_t maxDistance = std::numeric_limits<number_t>::max(),
                     number_t comparisonConditioner = 1e-3, bool directed = false,
                     number_t maxEdgeCost = std::numeric_limits<number_t>::max());

  static void computeTree(AdjacencyMap& amap);
  static void visitAdjacencyMap(AdjacencyMap& amap, TreeAction* action, bool useDistance = false);
  static void connectedSubset(HyperGraph::VertexSet& connected, HyperGraph::VertexSet& visited,
                              HyperGraph::VertexSet& startingSet,
                              const std::shared_ptr<HyperGraph>& g,
                              const std::shared_ptr<HyperGraph::Vertex>& v,
                              HyperDijkstra::CostFunction* cost, number_t distance,
                              number_t comparisonConditioner,
                              number_t maxEdgeCost = std::numeric_limits<number_t>::max());

 protected:
  void reset();

  AdjacencyMap _adjacencyMap;
  HyperGraph::VertexSet _visited;
  std::shared_ptr<HyperGraph> _graph;
};

struct G2O_CORE_API UniformCostFunction : public HyperDijkstra::CostFunction {
  virtual number_t operator()(HyperGraph::Edge* edge, HyperGraph::Vertex* from,
                              HyperGraph::Vertex* to);
};

}  // namespace g2o
#endif
