// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_AIS_GENERAL_DIJKSTRA_HH
#define G2O_AIS_GENERAL_DIJKSTRA_HH

#include <map>
#include <set>
#include <limits>

#include "hyper_graph.h"

namespace g2o{

  struct G2O_CORE_API HyperDijkstra{
    struct G2O_CORE_API CostFunction {
      virtual double operator() (HyperGraph::Edge* e, HyperGraph::Vertex* from, HyperGraph::Vertex* to)=0;
    };

    struct G2O_CORE_API TreeAction {
      virtual double perform(HyperGraph::Vertex* v, HyperGraph::Vertex* vParent, HyperGraph::Edge* e);
      virtual double perform(HyperGraph::Vertex* v, HyperGraph::Vertex* vParent, HyperGraph::Edge* e, double distance);
    };

    
    struct G2O_CORE_API AdjacencyMapEntry{
      friend struct HyperDijkstra;
      AdjacencyMapEntry(HyperGraph::Vertex* _child=0, 
          HyperGraph::Vertex* _parent=0, 
          HyperGraph::Edge* _edge=0, 
          double _distance=std::numeric_limits<double>::max());
      HyperGraph::Vertex* child() const {return _child;}
      HyperGraph::Vertex* parent() const {return _parent;}
      HyperGraph::Edge* edge() const {return _edge;}
      double distance() const {return _distance;}
      HyperGraph::VertexSet& children() {return _children;}
      const HyperGraph::VertexSet& children() const {return _children;}
      protected:
      HyperGraph::Vertex* _child;
      HyperGraph::Vertex* _parent;
      HyperGraph::Edge* _edge;
      double _distance;
      HyperGraph::VertexSet _children;
    };

    typedef std::map<HyperGraph::Vertex*, AdjacencyMapEntry> AdjacencyMap;
    HyperDijkstra(HyperGraph* g);
    HyperGraph::VertexSet& visited() {return _visited; }
    AdjacencyMap& adjacencyMap() {return _adjacencyMap; }
    HyperGraph* graph() {return _graph;} 

    void shortestPaths(HyperGraph::Vertex* v, 
           HyperDijkstra::CostFunction* cost, 
           double maxDistance=std::numeric_limits< double >::max(), 
           double comparisonConditioner=1e-3, 
           bool directed=false,
           double maxEdgeCost=std::numeric_limits< double >::max());

    void shortestPaths(HyperGraph::VertexSet& vset, 
           HyperDijkstra::CostFunction* cost, 
           double maxDistance=std::numeric_limits< double >::max(), 
           double comparisonConditioner=1e-3, 
           bool directed=false,
           double maxEdgeCost=std::numeric_limits< double >::max());


    static void computeTree(AdjacencyMap& amap);
    static void visitAdjacencyMap(AdjacencyMap& amap, TreeAction* action, bool useDistance=false);
    static void connectedSubset(HyperGraph::VertexSet& connected, HyperGraph::VertexSet& visited, 
           HyperGraph::VertexSet& startingSet, 
           HyperGraph* g, HyperGraph::Vertex* v,
           HyperDijkstra::CostFunction* cost, double distance, double comparisonConditioner,
           double maxEdgeCost=std::numeric_limits< double >::max() );

  protected:
    void reset();

    AdjacencyMap _adjacencyMap;
    HyperGraph::VertexSet _visited;
    HyperGraph* _graph;
  };

  struct G2O_CORE_API UniformCostFunction: public HyperDijkstra::CostFunction {
    virtual double operator ()(HyperGraph::Edge* edge, HyperGraph::Vertex* from, HyperGraph::Vertex* to);
  };

}
#endif
