// g2o - General Graph Optimization
// Copyright (C) 2011 G. Grisetti, R. Kuemmerle, W. Burgard
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

#include "cache.h"

#include <iostream>
#include <utility>

#include "factory.h"
#include "optimizable_graph.h"

namespace g2o {

Cache::CacheKey::CacheKey(std::string type_, ParameterVector parameters_)
    : type_(std::move(type_)), parameters_(std::move(parameters_)) {}

Cache::Cache(CacheContainer* container, ParameterVector parameters)
    : parameters_(std::move(parameters)), container_(container) {}

bool Cache::CacheKey::operator<(const Cache::CacheKey& c) const {
  if (type_ < c.type_) return true;
  if (c.type_ < type_) return false;
  return std::lexicographical_compare(parameters_.begin(), parameters_.end(),
                                      c.parameters_.begin(),
                                      c.parameters_.end());
}

const OptimizableGraph::Vertex& Cache::vertex() const {
  return container_->vertex();
}

const ParameterVector& Cache::parameters() const { return parameters_; }

Cache::CacheKey Cache::key() const {
  Factory* factory = Factory::instance();
  return CacheKey(factory->tag(this), parameters_);
};

void Cache::update() {
  if (!updateNeeded_) return;
  updateImpl();
  updateNeeded_ = false;
}

CacheContainer::CacheContainer(const OptimizableGraph::Vertex& vertex)
    : vertex_{vertex} {}

std::shared_ptr<Cache> CacheContainer::findCache(const Cache::CacheKey& key) {
  auto it = find(key);
  if (it == end()) return nullptr;
  return it->second;
}

std::shared_ptr<Cache> CacheContainer::createCache(const Cache::CacheKey& key) {
  Factory* f = Factory::instance();
  std::unique_ptr<HyperGraph::HyperGraphElement> e = f->construct(key.type());
  if (!e) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    std::cerr << "fatal error in creating cache of type " << key.type()
              << std::endl;
    return nullptr;
  }
  auto c = std::shared_ptr<Cache>(dynamic_cast<Cache*>(e.release()));
  if (!c) {
    std::cerr << __PRETTY_FUNCTION__ << std::endl;
    std::cerr << "fatal error in creating cache of type " << key.type()
              << std::endl;
    return nullptr;
  }
  c->container_ = this;
  c->parameters_ = key.parameters_;
  insert(make_pair(key, c));
  c->update();
  return c;
}

const OptimizableGraph::Vertex& CacheContainer::vertex() const { return vertex_; }

void CacheContainer::update() {
  for (auto& it : *this) {
    (it.second)->update();
  }
  updateNeeded_ = false;
}

void CacheContainer::setUpdateNeeded(bool needUpdate) {
  updateNeeded_ = needUpdate;
  for (auto& it : *this) {
    (it.second)->updateNeeded_ = needUpdate;
  }
}

}  // namespace g2o
