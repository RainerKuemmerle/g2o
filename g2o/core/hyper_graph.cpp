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

#include "hyper_graph.h"

#include <assert.h>
#include <queue>

namespace g2o {

  HyperGraph::Vertex::Vertex(int id) : _id(id)
  {
  }

  HyperGraph::Vertex::~Vertex()
  {
  }

  HyperGraph::Edge::Edge(int id) : _id(id)
  {
  }

  HyperGraph::Edge::~Edge()
  {
  }

  void HyperGraph::Edge::resize(size_t size)
  {
    _vertices.resize(size, 0);
  }

  void HyperGraph::Edge::setId(int id)
  {
    _id = id;
  }

  HyperGraph::Vertex* HyperGraph::vertex(int id)
  {
    VertexIDMap::iterator it=_vertices.find(id);
    if (it==_vertices.end())
      return 0;
    return it->second;
  }

  const HyperGraph::Vertex* HyperGraph::vertex(int id) const
  {
    VertexIDMap::const_iterator it=_vertices.find(id);
    if (it==_vertices.end())
      return 0;
    return it->second;
  }

  HyperGraph::Vertex* HyperGraph::addVertex(Vertex* v)
  {
    Vertex* vn=vertex(v->id());
    if (vn)
      return 0;
    _vertices.insert( std::make_pair(v->id(),v) );
    return v;
  }

  HyperGraph::Edge* HyperGraph::addEdge(Edge* e)
  {
    std::pair<EdgeSet::iterator, bool> result = _edges.insert(e);
    if (! result.second)
      return 0;
    for (std::vector<Vertex*>::iterator it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      Vertex* v = *it;
      v->edges().insert(e);
    }
    return e;
  }

  bool HyperGraph::removeVertex(Vertex* v)
  {
    VertexIDMap::iterator it=_vertices.find(v->id());
    if (it==_vertices.end())
      return false;
    assert(it->second==v);
    //remove all edges which are entering or leaving v;
    EdgeSet tmp(v->edges());
    for (EdgeSet::iterator it=tmp.begin(); it!=tmp.end(); ++it){
      if (!removeEdge(*it)){
        assert(0);
      }
    }
    _vertices.erase(it);
    delete v;
    return true;
  }

  bool HyperGraph::removeEdge(Edge* e)
  {
    EdgeSet::iterator it = _edges.find(e);
    if (it == _edges.end())
      return false;
    _edges.erase(it);

    for (std::vector<Vertex*>::iterator vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
      Vertex* v = *vit;
      it = v->edges().find(e);
      assert(it!=v->edges().end());
      v->edges().erase(it);
    }

    delete e;
    return true;
  }

  HyperGraph::HyperGraph()
  {
  }

  void HyperGraph::clear()
  {
    for (VertexIDMap::iterator it=_vertices.begin(); it!=_vertices.end(); ++it)
      delete (it->second);
    for (EdgeSet::iterator it=_edges.begin(); it!=_edges.end(); ++it)
      delete (*it);
    _vertices.clear();
    _edges.clear();
  }

  HyperGraph::~HyperGraph()
  {
    clear();
  }

} // end namespace
