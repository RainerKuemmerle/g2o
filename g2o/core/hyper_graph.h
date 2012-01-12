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

#ifndef G2O_AIS_HYPER_GRAPH_HH
#define G2O_AIS_HYPER_GRAPH_HH

#include <map>
#include <set>
#include <bitset>
#include <cassert>
#include <vector>
#include <limits>
#include <cstddef>

#ifdef _MSC_VER
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif

/** @addtogroup graph */
//@{
namespace g2o {

  /**
     Class that models a directed  Hyper-Graph. An hyper graph is a graph where an edge
     can connect one or more nodes. Both Vertices and Edges of an hyoper graph
     derive from the same class HyperGraphElement, thus one can implement generic algorithms
     that operate transparently on edges or vertices (see HyperGraphAction).

     The vertices are uniquely identified by an int id, while the edges are
     identfied by their pointers. 
   */
  class HyperGraph
  {
    public:

      /**
       * \brief enum of all the types we have in our graphs
       */
      enum HyperGraphElementType {
        HGET_VERTEX,
        HGET_EDGE,
        HGET_PARAMETER,
        HGET_CACHE,
        HGET_DATA,
        HGET_NUM_ELEMS // keep as last elem
      };

      typedef std::bitset<HyperGraph::HGET_NUM_ELEMS> GraphElemBitset;

      class Vertex;
      class Edge;
      
      /**
       * base hyper graph element, specialized in vertex and edge
       */
      struct HyperGraphElement {
        virtual ~HyperGraphElement() {}
        /**
         * returns the type of the graph element, see HyperGraphElementType
         */
        virtual HyperGraphElementType elementType() const = 0;
      };

      typedef std::set<Edge*>                           EdgeSet;
      typedef std::set<Vertex*>                         VertexSet;

      typedef std::tr1::unordered_map<int, Vertex*>     VertexIDMap;
      typedef std::vector<Vertex*>                      VertexVector;

      //! abstract Vertex, your types must derive from that one
      class Vertex : public HyperGraphElement {
        public:
          //! creates a vertex having an ID specified by the argument
          explicit Vertex(int id=-1);
          virtual ~Vertex();
          //! returns the id
          int id() const {return _id;}
          //! returns the set of hyper-edges that are leaving/entering in this vertex
          const EdgeSet& edges() const {return _edges;}
          //! returns the set of hyper-edges that are leaving/entering in this vertex
          EdgeSet& edges() {return _edges;}
          virtual HyperGraphElementType elementType() const { return HGET_VERTEX;}
        protected:
          int _id;
          EdgeSet _edges;
      };

      /** 
       * Abstract Edge class. Your nice edge classes should inherit from that one.
       * An hyper-edge has pointers to the vertices it connects and stores them in a vector.
       */
      class Edge : public HyperGraphElement {
        public:
          //! creates and empty edge with no vertices
          explicit Edge(int id = -1);
          virtual ~Edge();

          /**
           * resizes the number of vertices connected by this edge
           */
          virtual void resize(size_t size);
          /**
            returns the vector of pointers to the vertices connected by the hyper-edge.
            */
          const VertexVector& vertices() const { return _vertices;}
          /**
            returns the vector of pointers to the vertices connected by the hyper-edge.
            */
          VertexVector& vertices() { return _vertices;}
          /**
            returns the pointer to the ith vertex connected to the hyper-edge.
            */
          const Vertex* vertex(size_t i) const { return _vertices[i];}
          /**
            returns the pointer to the ith vertex connected to the hyper-edge.
            */
          Vertex* vertex(size_t i) { return _vertices[i];}
          /**
            set the ith vertex on the hyper-edge to the pointer supplied
            */
          void setVertex(size_t i, Vertex* v) { _vertices[i]=v;}

          int id() const {return _id;}
          void setId(int id);
          virtual HyperGraphElementType elementType() const { return HGET_EDGE;}
        protected:
          VertexVector _vertices;
          int _id; ///< unique id
      };

    public:
      //! constructs an empty hyper graph
      HyperGraph();
      //! destroys the hyper-graph and all the vertices of the graph
      virtual ~HyperGraph();

      //! returns a vertex <i>id</i> in the hyper-graph, or 0 if the vertex id is not present
      Vertex* vertex(int id);
      //! returns a vertex <i>id</i> in the hyper-graph, or 0 if the vertex id is not present
      const Vertex* vertex(int id) const;

      //! removes a vertex from the graph. Returns true on success (vertex was present)
      virtual bool removeVertex(Vertex* v);
      //! removes a vertex from the graph. Returns true on success (edge was present)
      virtual bool removeEdge(Edge* e);
      //! clears the graph and empties all structures.
      virtual void clear();

      //! @returns the map <i>id -> vertex</i> where the vertices are stored
      const VertexIDMap& vertices() const {return _vertices;}
      //! @returns the map <i>id -> vertex</i> where the vertices are stored
      VertexIDMap& vertices() {return _vertices;}

      //! @returns the set of edges of the hyper graph
      const EdgeSet& edges() const {return _edges;}
      //! @returns the set of edges of the hyper graph
      EdgeSet& edges() {return _edges;}

      /**
       * adds a vertex to the graph. The id of the vertex should be set before
       * invoking this function. the function fails if another vertex
       * with the same id is already in the graph.
       * returns a poiner to the vertex, on success, or 0 on failure.
       */
      Vertex* addVertex(Vertex* v);

      //! adds an edge  to the graph. If the edge is already in the graph, it does nothing and returns 0. otherwise it returns <i>e</i>.
      Edge* addEdge(Edge* e);

    protected:
      VertexIDMap _vertices;
      EdgeSet _edges;
  };

} // end namespace

//@}

#endif
