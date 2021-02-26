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

#include <assert.h>

#include <iterator>
#include <queue>
#include <unordered_set>

#include "ownership.h"

namespace g2o {

static std::shared_ptr<HyperGraph::Vertex> kNonExistantVertex(nullptr);

HyperGraph::Data::Data() {
  _next = 0;
  _dataContainer = 0;
}

HyperGraph::Data::~Data() {}

HyperGraph::Vertex::Vertex(int id) : _id(id) {}

HyperGraph::Vertex::~Vertex() {}

HyperGraph::Edge::Edge(int id) : _id(id) {}

HyperGraph::Edge::~Edge() {}

int HyperGraph::Edge::numUndefinedVertices() const {
  return std::count_if(_vertices.begin(), _vertices.end(),
                       [](const std::shared_ptr<Vertex>& ptr) { return ptr.get() == nullptr; });
}

void HyperGraph::Edge::resize(size_t size) { _vertices.resize(size, 0); }

void HyperGraph::Edge::setId(int id) { _id = id; }

std::shared_ptr<HyperGraph::Vertex> HyperGraph::vertex(int id) {
  VertexIDMap::iterator it = _vertices.find(id);
  if (it == _vertices.end()) return std::shared_ptr<HyperGraph::Vertex>();
  return it->second;
}

std::shared_ptr<const HyperGraph::Vertex> HyperGraph::vertex(int id) const {
  VertexIDMap::const_iterator it = _vertices.find(id);
  if (it == _vertices.end()) return std::shared_ptr<HyperGraph::Vertex>();
  return it->second;
}

bool HyperGraph::addVertex(std::shared_ptr<Vertex>& v) {
  auto result = _vertices.insert(std::make_pair(v->id(), v));
  return result.second;
}

/**
 * changes the id of a vertex already in the graph, and updates the bookkeeping
 @ returns false if the vertex is not in the graph;
*/
bool HyperGraph::changeId(std::shared_ptr<Vertex>& v, int newId) {
  auto v2 = vertex(v->id());
  if (v != v2) return false;
  _vertices.erase(v->id());
  v->setId(newId);
  _vertices.insert(std::make_pair(v->id(), v));
  return true;
}

bool HyperGraph::addEdge(std::shared_ptr<Edge>& e) {
  for (const auto& v : e->vertices()) {  // be sure that all vertices are set
    if (!v) return false;
  }

  // check for duplicates in the vertices and do not add this edge
  if (e->vertices().size() == 2) {
    if (e->vertices()[0] == e->vertices()[1]) return false;
  } else if (e->vertices().size() == 3) {
    if (e->vertices()[0] == e->vertices()[1] || e->vertices()[0] == e->vertices()[2] ||
        e->vertices()[1] == e->vertices()[2])
      return false;
  } else if (e->vertices().size() > 3) {
    std::unordered_set<Vertex*> vertexPointer;
    for (const auto& v : e->vertices()) vertexPointer.insert(v.get());
    if (vertexPointer.size() != e->vertices().size()) return false;
  }

  std::pair<EdgeSet::iterator, bool> result = _edges.insert(e);
  if (!result.second) return false;

  for (auto& v : e->vertices()) {  // connect the vertices to this edge
    v->edges().insert(e);
  }

  return true;
}

bool HyperGraph::setEdgeVertex(std::shared_ptr<Edge>& e, int pos, std::shared_ptr<Vertex>& v) {
  auto vOld = e->vertex(pos);
  if (vOld) vOld->edges().erase(e);
  e->setVertex(pos, v);
  if (v) v->edges().insert(e);
  return true;
}

bool HyperGraph::mergeVertices(std::shared_ptr<Vertex>& vBig, std::shared_ptr<Vertex>& vSmall,
                               bool erase) {
  VertexIDMap::iterator it = _vertices.find(vBig->id());
  if (it == _vertices.end()) return false;

  it = _vertices.find(vSmall->id());
  if (it == _vertices.end()) return false;

  EdgeSetWeak tmp = vSmall->edges();
  bool ok = true;
  for (EdgeSetWeak::iterator it = tmp.begin(); it != tmp.end(); ++it) {
    std::shared_ptr<HyperGraph::Edge> e = it->lock();
    for (size_t i = 0; i < e->vertices().size(); i++) {
      if (e->vertex(i) == vSmall) {
        ok &= setEdgeVertex(e, i, vBig);
      }
    }
  }
  if (erase) removeVertex(vSmall);
  return ok;
}

bool HyperGraph::detachVertex(const std::shared_ptr<Vertex>& v) {
  VertexIDMap::iterator it = _vertices.find(v->id());
  if (it == _vertices.end()) return false;
  assert(it->second == v);
  EdgeSetWeak tmp = v->edges();
  for (EdgeSetWeak::iterator it = tmp.begin(); it != tmp.end(); ++it) {
    std::shared_ptr<HyperGraph::Edge> e = it->lock();
    for (size_t i = 0; i < e->vertices().size(); i++) {
      if (v == e->vertex(i)) setEdgeVertex(e, i, kNonExistantVertex);
    }
  }
  return true;
}

bool HyperGraph::removeVertex(const std::shared_ptr<Vertex>& v, bool detach) {
  if (detach) {
    bool result = detachVertex(v);
    if (!result) {
      assert(0 && "inconsistency in detaching vertex");
    }
  }
  VertexIDMap::iterator it = _vertices.find(v->id());
  if (it == _vertices.end()) return false;
  assert(it->second == v);
  // remove all edges which are entering or leaving v;
  EdgeSetWeak tmp = v->edges();
  for (EdgeSetWeak::iterator it = tmp.begin(); it != tmp.end(); ++it) {
    if (!removeEdge(it->lock())) {
      assert(0 && "error in erasing vertex");
    }
  }
  _vertices.erase(it);
  return true;
}

bool HyperGraph::removeEdge(const std::shared_ptr<Edge>& e) {
  EdgeSet::iterator it = _edges.find(e);
  if (it == _edges.end()) return false;
  _edges.erase(it);
  for (auto vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
    auto& v = *vit;
    if (!v) continue;
    EdgeSetWeak::iterator foundIt = v->edges().find(e);
    assert(foundIt != v->edges().end());
    v->edges().erase(foundIt);
  }
  return true;
}

HyperGraph::HyperGraph() {}

void HyperGraph::clear() {
  _vertices.clear();
  _edges.clear();
}

HyperGraph::~HyperGraph() { HyperGraph::clear(); }

}  // namespace g2o
