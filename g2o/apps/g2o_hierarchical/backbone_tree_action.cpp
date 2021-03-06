// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "backbone_tree_action.h"

#include <limits>

#include "edge_types_cost_function.h"
#include "g2o/core/factory.h"
namespace g2o {

using namespace std;

BackBoneTreeAction::BackBoneTreeAction(SparseOptimizer* optimizer, const std::string& vertexTag,
                                       int level, int step)
    : _optimizer(optimizer), _vertexTag(vertexTag), _level(level), _step(step) {
  _factory = Factory::instance();
  init();
}

void BackBoneTreeAction::init() {
  _vsMap.clear();
  _vsMmap.clear();
  _freeEdges.clear();
  for (auto it = _optimizer->edges().begin(); it != _optimizer->edges().end(); ++it) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(*it);
    if (e->level() == _level) {
      _freeEdges.insert(e);
    }
  }
}

double BackBoneTreeAction::perform(const std::shared_ptr<HyperGraph::Vertex>& v,
                                   const std::shared_ptr<HyperGraph::Vertex>& vParent,
                                   const std::shared_ptr<HyperGraph::Edge>& e, double distance) {
  int depth = (int)distance;
  if (_factory->tag(v.get()) != _vertexTag) return 0;
  auto vParentCast = std::static_pointer_cast<OptimizableGraph::Vertex>(vParent);
  auto parentStar = getStar(vParentCast);
  if (!parentStar) {
    parentStar = std::make_shared<Star>(_level + 1, _optimizer);
    addToMap(parentStar, vParentCast);
    parentStar->_gauge.insert(vParentCast);
  }
  auto vCast = std::static_pointer_cast<OptimizableGraph::Vertex>(v);
  addToMap(parentStar, vCast);
  auto eCast = std::static_pointer_cast<OptimizableGraph::Edge>(e);
  fillStar(parentStar, eCast);

  // every _step levels you go down in the tree, create a new star
  if (depth && !(depth % _step)) {
    auto star = std::make_shared<Star>(_level + 1, _optimizer);
    addToMap(star, vCast);
    star->_gauge.insert(vCast);
  }
  return 1;
}

void BackBoneTreeAction::addToMap(const std::shared_ptr<Star>& s,
                                  const std::shared_ptr<OptimizableGraph::Vertex>& v) {
  VertexStarMap::iterator it = _vsMap.find(v);
  if (it != _vsMap.end())
    it->second = s;
  else
    _vsMap.insert(make_pair(v, s));
  _vsMmap.insert(make_pair(v, s));
  s->_lowLevelVertices.insert(v);
}

std::shared_ptr<Star> BackBoneTreeAction::getStar(
    const std::shared_ptr<OptimizableGraph::Vertex>& v) {
  VertexStarMap::iterator it = _vsMap.find(v);
  if (it == _vsMap.end()) return nullptr;
  return it->second;
}

bool BackBoneTreeAction::fillStar(const std::shared_ptr<Star>& s,
                                  const std::shared_ptr<OptimizableGraph::Edge>& e) {
  HyperGraph::EdgeSet::iterator it = _freeEdges.find(e);
  if (it != _freeEdges.end()) {
    _freeEdges.erase(it);
    s->_lowLevelEdges.insert(e);
    for (size_t i = 0; i < e->vertices().size(); i++) {
      s->_lowLevelVertices.insert(e->vertices()[i]);
    }
    return true;
  }
  return false;
}
}  // namespace g2o
