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

#include "hyper_graph.h"

#include <algorithm>
#include <cassert>
#include <memory>
#include <unordered_set>
#include <utility>

#include "g2o/stuff/hash_combine.h"

namespace g2o {

HyperGraph::Vertex::Vertex(int id) : id_(id) {}

HyperGraph::Vertex::~Vertex() = default;

HyperGraph::Edge::Edge(int id) : id_(id) {}

HyperGraph::Edge::~Edge() = default;

int HyperGraph::Edge::numUndefinedVertices() const {
  return std::count_if(
      vertices_.begin(), vertices_.end(),
      [](const std::shared_ptr<Vertex>& ptr) { return ptr == nullptr; });
}

void HyperGraph::Edge::resize(size_t size) { vertices_.resize(size, nullptr); }

void HyperGraph::Edge::setId(int id) { id_ = id; }

const HyperGraph::DataContainer::DataVector&
HyperGraph::DataContainer::userData() const {
  static HyperGraph::DataContainer::DataVector kEmptyDataContainer;
  return container_.get() ? *container_ : kEmptyDataContainer;
}

HyperGraph::DataContainer::DataVector& HyperGraph::DataContainer::userData() {
  allocate();
  return *container_;
}

void HyperGraph::DataContainer::setUserData(const std::shared_ptr<Data>& obs) {
  allocate();
  container_->clear();
  addUserData(obs);
  container_->shrink_to_fit();
}

void HyperGraph::DataContainer::addUserData(const std::shared_ptr<Data>& obs) {
  allocate();
  container_->emplace_back(obs);
}

void HyperGraph::DataContainer::allocate() {
  if (container_) return;
  container_ = std::make_unique<HyperGraph::DataContainer::DataVector>();
}

HyperGraph::VertexIDEdges::LookupResult HyperGraph::VertexIDEdges::lookup(
    int id) const {
  auto found = lookup_.find(id);
  const bool found_state = found != lookup_.end();
  return std::make_pair(found, found_state);
}

std::shared_ptr<HyperGraph::Vertex> HyperGraph::vertex(int id) {
  auto it = vertices_.find(id);
  if (it == vertices_.end()) return std::shared_ptr<HyperGraph::Vertex>();
  return it->second;
}

std::shared_ptr<const HyperGraph::Vertex> HyperGraph::vertex(int id) const {
  auto it = vertices_.find(id);
  if (it == vertices_.end()) return std::shared_ptr<HyperGraph::Vertex>();
  return it->second;
}

bool HyperGraph::addVertex(const std::shared_ptr<Vertex>& v) {
  auto result = vertices_.emplace(v->id(), v);
  return result.second;
}

/**
 * changes the id of a vertex already in the graph, and updates the bookkeeping
 @ returns false if the vertex is not in the graph;
*/
bool HyperGraph::changeId(std::shared_ptr<Vertex>& v, int newId) {
  auto v2 = vertex(v->id());
  if (v != v2) return false;
  vertices_.erase(v->id());
  v->setId(newId);
  vertices_.emplace(v->id(), v);
  return true;
}

bool HyperGraph::addEdge(const std::shared_ptr<Edge>& e) {
  for (const auto& v : e->vertices()) {  // be sure that all vertices are set
    if (!v) return false;
  }

  // check for duplicates in the vertices and do not add this edge
  if (e->vertices().size() == 2) {
    if (e->vertices()[0] == e->vertices()[1]) return false;
  } else if (e->vertices().size() == 3) {
    if (e->vertices()[0] == e->vertices()[1] ||
        e->vertices()[0] == e->vertices()[2] ||
        e->vertices()[1] == e->vertices()[2])
      return false;
  } else if (e->vertices().size() > 3) {
    std::unordered_set<Vertex*> vertexPointer;
    for (const auto& v : e->vertices()) vertexPointer.insert(v.get());
    if (vertexPointer.size() != e->vertices().size()) return false;
  }

  const std::pair<EdgeSet::iterator, bool> result = edges_.emplace(e);
  return result.second;
}

bool HyperGraph::setEdgeVertex(const std::shared_ptr<Edge>& e, int pos,
                               const std::shared_ptr<Vertex>& v) {
  e->setVertex(pos, v);
  return true;
}

bool HyperGraph::removeVertex(const std::shared_ptr<Vertex>& v) {
  auto it = vertices_.find(v->id());
  if (it == vertices_.end()) return false;
  vertices_.erase(it);
  return true;
}

bool HyperGraph::removeEdge(const std::shared_ptr<Edge>& e) {
  auto it = edges_.find(e);
  if (it == edges_.end()) return false;
  edges_.erase(it);
  return true;
}

HyperGraph::HyperGraph() = default;

void HyperGraph::clear() {
  vertices_.clear();
  edges_.clear();
}

HyperGraph::~HyperGraph() { HyperGraph::clear(); }

HyperGraph::VertexIDEdges HyperGraph::createVertexEdgeLookup() const {
  HyperGraph::VertexIDEdges result;
  result.hash_ = hash();
  for (const auto& e : edges_) {
    auto oe = std::static_pointer_cast<Edge>(e);
    for (const auto& v : e->vertices()) {
      result.lookup_[v->id()].push_back(oe);
    }
  }
  return result;
}

std::size_t HyperGraph::hash() const {
  std::size_t result = 0x42;
  for (const auto& v : vertices_) {
    hash_combine(result, v.first);
    hash_combine(result, v.second.get());
  }
  for (const auto& e : edges_) {
    hash_combine(result, e.get());
    for (const auto& v : e->vertices()) {
      hash_combine(result, v.get());
    }
  }
  return result;
}

}  // namespace g2o
