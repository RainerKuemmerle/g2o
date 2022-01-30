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
#include <utility>

#include "edge_types_cost_function.h"
#include "g2o/core/factory.h"
namespace g2o {

BackBoneTreeAction::BackBoneTreeAction(SparseOptimizer* optimizer,
                                       std::string vertexTag, int level,
                                       int step)
    : optimizer_(optimizer),
      vertexTag_(std::move(vertexTag)),
      level_(level),
      step_(step) {
  factory_ = Factory::instance();
  init();
}

void BackBoneTreeAction::init() {
  vsMap_.clear();
  vsMmap_.clear();
  freeEdges_.clear();
  for (const auto& it : optimizer_->edges()) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(it);
    if (e->level() == level_) {
      freeEdges_.insert(e);
    }
  }
}

double BackBoneTreeAction::perform(
    const std::shared_ptr<HyperGraph::Vertex>& v,
    const std::shared_ptr<HyperGraph::Vertex>& vParent,
    const std::shared_ptr<HyperGraph::Edge>& e, double distance) {
  int depth = static_cast<int>(distance);
  if (factory_->tag(v.get()) != vertexTag_) return 0;
  auto vParentCast =
      std::static_pointer_cast<OptimizableGraph::Vertex>(vParent);
  auto parentStar = getStar(vParentCast);
  if (!parentStar) {
    parentStar = std::make_shared<Star>(level_ + 1, optimizer_);
    addToMap(parentStar, vParentCast);
    parentStar->gauge().insert(vParentCast);
  }
  auto vCast = std::static_pointer_cast<OptimizableGraph::Vertex>(v);
  addToMap(parentStar, vCast);
  auto eCast = std::static_pointer_cast<OptimizableGraph::Edge>(e);
  fillStar(parentStar, eCast);

  // every _step levels you go down in the tree, create a new star
  if (depth && !(depth % step_)) {
    auto star = std::make_shared<Star>(level_ + 1, optimizer_);
    addToMap(star, vCast);
    star->gauge().insert(vCast);
  }
  return 1;
}

void BackBoneTreeAction::addToMap(
    const std::shared_ptr<Star>& s,
    const std::shared_ptr<OptimizableGraph::Vertex>& v) {
  auto it = vsMap_.find(v);
  if (it != vsMap_.end())
    it->second = s;
  else
    vsMap_.insert(make_pair(v, s));
  vsMmap_.insert(make_pair(v, s));
  s->lowLevelVertices().insert(v);
}

std::shared_ptr<Star> BackBoneTreeAction::getStar(
    const std::shared_ptr<OptimizableGraph::Vertex>& v) {
  auto it = vsMap_.find(v);
  if (it == vsMap_.end()) return nullptr;
  return it->second;
}

bool BackBoneTreeAction::fillStar(
    const std::shared_ptr<Star>& s,
    const std::shared_ptr<OptimizableGraph::Edge>& e) {
  auto it = freeEdges_.find(e);
  if (it != freeEdges_.end()) {
    freeEdges_.erase(it);
    s->lowLevelEdges().insert(e);
    for (auto& i : e->vertices()) {
      s->lowLevelVertices().insert(i);
    }
    return true;
  }
  return false;
}
}  // namespace g2o
