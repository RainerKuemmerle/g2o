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

#ifndef G2O_AIS_HYPER_GRAPH_HH
#define G2O_AIS_HYPER_GRAPH_HH

#include <bitset>
#include <cassert>
#include <cstddef>
#include <memory>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "g2o_core_api.h"

/** @addtogroup graph */
//@{
namespace g2o {

/**
   Class that models a directed  Hyper-Graph. An hyper graph is a graph where an
   edge can connect one or more nodes. Both Vertices and Edges of an hyper graph
   derive from the same class HyperGraphElement, thus one can implement generic
   algorithms that operate transparently on edges or vertices (see
   HyperGraphAction).

   The vertices are uniquely identified by an int id, while the edges are
   identfied by their pointers.
 */
class G2O_CORE_API HyperGraph {
 public:
  /**
   * \brief enum of all the types we have in our graphs
   */
  enum G2O_CORE_API HyperGraphElementType {
    kHgetVertex,
    kHgetEdge,
    kHgetParameter,
    kHgetCache,
    kHgetData,
    kHgetNumElems  // keep as last elem
  };

  enum G2O_CORE_API HyperGraphDefaultIds {
    kUnassignedId = -1,
    kInvalidId = -2
  };

  using GraphElemBitset = std::bitset<HyperGraph::kHgetNumElems>;

  class G2O_CORE_API Data;
  class G2O_CORE_API DataContainer;
  class G2O_CORE_API Vertex;
  class G2O_CORE_API Edge;

  /**
   * base hyper graph element, specialized in vertex and edge
   */
  struct G2O_CORE_API HyperGraphElement {
    virtual ~HyperGraphElement() = default;
    /**
     * returns the type of the graph element, see HyperGraphElementType
     */
    virtual HyperGraphElementType elementType() const = 0;
  };

  /**
   * \brief data packet for a vertex. Extend this class to store in the vertices
   * the potential additional information you need (e.g. images, laser scans,
   * ...).
   */
  class G2O_CORE_API Data : public HyperGraph::HyperGraphElement {
   public:
    Data();
    ~Data() override;
    //! read the data from a stream
    virtual bool read(std::istream& is) = 0;
    //! write the data to a stream
    virtual bool write(std::ostream& os) const = 0;
    HyperGraph::HyperGraphElementType elementType() const override {
      return HyperGraph::kHgetData;
    }
    std::shared_ptr<Data> next() const { return next_; }
    void setNext(std::shared_ptr<Data> next) { next_ = std::move(next); }
    std::shared_ptr<DataContainer> dataContainer() const {
      return dataContainer_;
    }
    void setDataContainer(std::shared_ptr<DataContainer> dataContainer) {
      dataContainer_ = std::move(dataContainer);
    }

   protected:
    std::shared_ptr<Data> next_;  // linked list of multiple data;
    std::shared_ptr<DataContainer> dataContainer_;
  };

  /**
   * \brief Container class that implements an interface for adding/removing
   Data elements in a linked list
   */
  class G2O_CORE_API DataContainer {
   public:
    //! the user data associated with this vertex
    std::shared_ptr<Data> userData() const { return userData_; }
    void setUserData(const std::shared_ptr<Data>& obs) { userData_ = obs; }
    void addUserData(const std::shared_ptr<Data>& obs) {
      if (obs) {
        obs->setNext(userData_);
        userData_ = obs;
      }
    }

   protected:
    std::shared_ptr<Data> userData_;
  };

  using EdgeSet = std::set<std::shared_ptr<Edge>>;
  using EdgeSetWeak =
      std::set<std::weak_ptr<Edge>, std::owner_less<std::weak_ptr<Edge>>>;
  using VertexSet = std::set<std::shared_ptr<Vertex>>;
  using VertexSetWeak =
      std::set<std::weak_ptr<Vertex>, std::owner_less<std::weak_ptr<Vertex>>>;

  using VertexIDMap = std::unordered_map<int, std::shared_ptr<Vertex>>;
  using VertexContainer = std::vector<std::shared_ptr<Vertex>>;
  using VertexContainerWeak = std::vector<std::weak_ptr<Vertex>>;

  //! abstract Vertex, your types must derive from that one
  class G2O_CORE_API Vertex : public HyperGraphElement {
   public:
    //! creates a vertex having an ID specified by the argument
    explicit Vertex(int id = kInvalidId);
    ~Vertex() override;
    //! returns the id
    int id() const { return id_; }
    virtual void setId(int newId) { id_ = newId; }
    //! returns the set of hyper-edges that are leaving/entering in this vertex
    const EdgeSetWeak& edges() const { return edges_; }
    //! returns the set of hyper-edges that are leaving/entering in this vertex
    EdgeSetWeak& edges() { return edges_; }
    HyperGraphElementType elementType() const override { return kHgetVertex; }

   protected:
    int id_;
    EdgeSetWeak edges_;
  };

  /**
   * Abstract Edge class. Your nice edge classes should inherit from that one.
   * An hyper-edge has pointers to the vertices it connects and stores them in a
   * vector.
   */
  class G2O_CORE_API Edge : public HyperGraphElement {
   public:
    //! creates and empty edge with no vertices
    explicit Edge(int id = kInvalidId);
    ~Edge() override;

    /**
     * resizes the number of vertices connected by this edge
     */
    virtual void resize(size_t size);
    /**
      returns the vector of pointers to the vertices connected by the
      hyper-edge.
      */
    const VertexContainer& vertices() const { return vertices_; }
    /**
      returns the vector of pointers to the vertices connected by the
      hyper-edge.
      */
    VertexContainer& vertices() { return vertices_; }
    /**
      returns the pointer to the ith vertex connected to the hyper-edge.
      */
    std::shared_ptr<const Vertex> vertex(size_t i) const {
      assert(i < vertices_.size() && "index out of bounds");
      return vertices_[i];
    }
    /**
      returns the pointer to the ith vertex connected to the hyper-edge.
      */
    std::shared_ptr<Vertex> vertex(size_t i) {
      assert(i < vertices_.size() && "index out of bounds");
      return vertices_[i];
    }
    /**
      set the ith vertex on the hyper-edge to the pointer supplied
      */
    void setVertex(size_t i, const std::shared_ptr<Vertex>& v) {
      assert(i < vertices_.size() && "index out of bounds");
      vertices_[i] = v;
    }

    int id() const { return id_; }
    void setId(int id);
    HyperGraphElementType elementType() const override { return kHgetEdge; }

    int numUndefinedVertices() const;

   protected:
    VertexContainer vertices_;
    int id_;  ///< unique id
  };

  //! constructs an empty hyper graph
  HyperGraph();
  //! destroys the hyper-graph and all the vertices of the graph
  virtual ~HyperGraph();

  //! returns a vertex <i>id</i> in the hyper-graph, or 0 if the vertex id is
  //! not present
  std::shared_ptr<Vertex> vertex(int id);
  //! returns a vertex <i>id</i> in the hyper-graph, or 0 if the vertex id is
  //! not present
  std::shared_ptr<const Vertex> vertex(int id) const;

  //! removes a vertex from the graph. Returns true on success (vertex was
  //! present)
  virtual bool removeVertex(const std::shared_ptr<Vertex>& v,
                            bool detach = false);
  //! removes a vertex from the graph. Returns true on success (edge was
  //! present)
  virtual bool removeEdge(const std::shared_ptr<Edge>& e);
  //! clears the graph and empties all structures.
  virtual void clear();

  //! @returns the map <i>id -> vertex</i> where the vertices are stored
  const VertexIDMap& vertices() const { return vertices_; }
  //! @returns the map <i>id -> vertex</i> where the vertices are stored
  VertexIDMap& vertices() { return vertices_; }

  //! @returns the set of edges of the hyper graph
  const EdgeSet& edges() const { return edges_; }
  //! @returns the set of edges of the hyper graph
  EdgeSet& edges() { return edges_; }

  /**
   * adds a vertex to the graph. The id of the vertex should be set before
   * invoking this function. the function fails if another vertex
   * with the same id is already in the graph.
   * returns true, on success, or false on failure.
   */
  virtual bool addVertex(const std::shared_ptr<Vertex>& v);

  /**
   * Adds an edge to the graph. If the edge is already in the graph, it
   * does nothing and returns false. Otherwise it returns true.
   */
  virtual bool addEdge(const std::shared_ptr<Edge>& e);

  /**
   * Sets the vertex in position "pos" within the edge and keeps the bookkeeping
   * consistent. If v ==0, the vertex is set to "invalid"
   */
  virtual bool setEdgeVertex(const std::shared_ptr<Edge>& e, int pos,
                             const std::shared_ptr<Vertex>& v);

  /**
   * merges two (valid) vertices, adjusts the bookkeeping and relabels all
   * edges. the observations of vSmall are retargeted to vBig. If erase = true,
   * vSmall is deleted from the graph repeatedly calls setEdgeVertex(...)
   */
  virtual bool mergeVertices(std::shared_ptr<Vertex>& vBig,
                             std::shared_ptr<Vertex>& vSmall, bool erase);

  /**
   * detaches a vertex from all connected edges
   */
  virtual bool detachVertex(const std::shared_ptr<Vertex>& v);

  /**
   * changes the id of a vertex already in the graph, and updates the
   bookkeeping
   @ returns false if the vertex is not in the graph;
   */
  virtual bool changeId(std::shared_ptr<Vertex>& v, int newId);

 protected:
  VertexIDMap vertices_;
  EdgeSet edges_;

 private:
  // Disable the copy constructor and assignment operator
  HyperGraph(const HyperGraph&) {}
  HyperGraph& operator=(const HyperGraph&) { return *this; }
};

}  // namespace g2o

//@}

#endif
