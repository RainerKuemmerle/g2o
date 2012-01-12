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

#ifndef G2O_ESTIMATE_PROPAGATOR_H
#define G2O_ESTIMATE_PROPAGATOR_H

#include "optimizable_graph.h"
#include "sparse_optimizer.h"

#include <map>
#include <set>
#include <limits>

#ifdef _MSC_VER
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif

namespace g2o {

  /**
   * \brief propagation of an initial guess
   */
  class EstimatePropagator {
    public:

      /**
       * \brief Applying the action for propagating.
       *
       * You may derive an own one, if necessary. The default is to call initialEstimate(from, to) for the edge.
       */
      struct PropagateAction {
        virtual void operator()(OptimizableGraph::Edge* e, const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) const
        {
          if (! to->fixed())
            e->initialEstimate(from, to);
        }
      };

      /**
       * \brief cost for traversing along active edges in the optimizer
       *
       * You may derive an own one, if necessary. The default is to return initialEstimatePossible(from, to) for the edge.
       */
      class PropagateCost {
        public:
          PropagateCost(SparseOptimizer* graph) : _graph(graph) {}
          virtual double operator()(OptimizableGraph::Edge* edge, const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to_) const
          {
            OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
            OptimizableGraph::Vertex* to = dynamic_cast<OptimizableGraph::Vertex*>(to_);
            SparseOptimizer::EdgeContainer::const_iterator it = _graph->findActiveEdge(e);
            if (it == _graph->activeEdges().end()) // it has to be an active edge
              return std::numeric_limits<double>::max();
            return e->initialEstimatePossible(from, to);
          }
        protected:
          SparseOptimizer* _graph;
      };

      class AdjacencyMapEntry;

      /**
       * \brief priority queue for AdjacencyMapEntry
       */
      class PriorityQueue : public std::multimap<double, AdjacencyMapEntry*> {
        public:
          void push(AdjacencyMapEntry* entry);
          AdjacencyMapEntry* pop();
      };

      /**
       * \brief data structure for loopuk during Dijkstra
       */
      class AdjacencyMapEntry {
        public:
          friend class EstimatePropagator;
          friend class PriorityQueue;
          AdjacencyMapEntry();
          void reset();
          OptimizableGraph::Vertex* child() const {return _child;}
          const OptimizableGraph::VertexSet& parent() const {return _parent;}
          OptimizableGraph::Edge* edge() const {return _edge;}
          double distance() const {return _distance;}
          int frontierLevel() const { return _frontierLevel;}

        protected:
          OptimizableGraph::Vertex* _child;
          OptimizableGraph::VertexSet _parent;
          OptimizableGraph::Edge* _edge;
          double _distance;
          int _frontierLevel;
        private: // for PriorityQueue
          bool inQueue;
          PriorityQueue::iterator queueIt;
      };

      /**
       * \brief hash function for a vertex
       */
      class VertexIDHashFunction {
        public:
          size_t operator ()(const OptimizableGraph::Vertex* v) const { return v->id();}
      };

      typedef std::tr1::unordered_map<OptimizableGraph::Vertex*, AdjacencyMapEntry, VertexIDHashFunction> AdjacencyMap;

    public:
      EstimatePropagator(OptimizableGraph* g);
      OptimizableGraph::VertexSet& visited() {return _visited; }
      AdjacencyMap& adjacencyMap() {return _adjacencyMap; }
      OptimizableGraph* graph() {return _graph;} 

      /**
       * propagate an initial guess starting from v. The function computes a spanning tree
       * whereas the cost for each edge is determined by calling cost() and the action applied to
       * each vertex is action().
       */
      void propagate(OptimizableGraph::Vertex* v, 
          const EstimatePropagator::PropagateCost& cost, 
          const EstimatePropagator::PropagateAction& action = PropagateAction(),
          double maxDistance=std::numeric_limits<double>::max(), 
          double maxEdgeCost=std::numeric_limits<double>::max());

      /**
       * same as above but starting to propagate from a set of vertices instead of just a single one.
       */
      void propagate(OptimizableGraph::VertexSet& vset, 
          const EstimatePropagator::PropagateCost& cost, 
          const EstimatePropagator::PropagateAction& action = PropagateAction(),
          double maxDistance=std::numeric_limits<double>::max(), 
          double maxEdgeCost=std::numeric_limits<double>::max());

    protected:
      void reset();

      AdjacencyMap _adjacencyMap;
      OptimizableGraph::VertexSet _visited;
      OptimizableGraph* _graph;
  };

}
#endif
