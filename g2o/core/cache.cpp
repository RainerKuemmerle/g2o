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
#include "optimizable_graph.h"
#include "factory.h"

#include <iostream>

namespace g2o {
  using namespace std;

  Cache::CacheKey::CacheKey() :
    _type(), _parameters()
  {
  }

  Cache::CacheKey::CacheKey(const std::string& type_, const ParameterVector& parameters_) :
    _type(type_), _parameters(parameters_)
  {
  }

  Cache::Cache(CacheContainer* container_, const ParameterVector& parameters_) :
    _updateNeeded(true), _parameters(parameters_), _container(container_)
  {
  }

  bool Cache::CacheKey::operator<(const Cache::CacheKey& c) const{
    if (_type < c._type)
      return true;
    else if (c._type < _type)
      return false;
    return std::lexicographical_compare (_parameters.begin( ), _parameters.end( ),
           c._parameters.begin( ), c._parameters.end( ) );
  }


  OptimizableGraph::Vertex* Cache::vertex() {
    if (container() )
      return container()->vertex();
    return nullptr;
  }

  OptimizableGraph* Cache::graph() {
    if (container())
      return container()->graph();
    return nullptr;
  }

  CacheContainer* Cache::container() {
    return _container;
  }

  ParameterVector& Cache::parameters() {
    return _parameters;
  }

  Cache::CacheKey Cache::key() const {
    Factory* factory=Factory::instance();
    return CacheKey(factory->tag(this), _parameters);
  };


  void Cache::update(){
    if (! _updateNeeded)
      return;
    updateImpl();
    _updateNeeded=false;
  }

  CacheContainer::CacheContainer(OptimizableGraph::Vertex* vertex_) : _updateNeeded(true) { _vertex = vertex_; }

  std::shared_ptr<Cache> CacheContainer::findCache(const Cache::CacheKey& key) {
    iterator it=find(key);
    if (it==end())
      return nullptr;
    return it->second;
  }

  std::shared_ptr<Cache> CacheContainer::createCache(const Cache::CacheKey& key){
    Factory* f = Factory::instance();
    std::unique_ptr<HyperGraph::HyperGraphElement> e = f->construct(key.type());
    if (!e) {
      cerr << __PRETTY_FUNCTION__ << endl;
      cerr << "fatal error in creating cache of type " << key.type() << endl;
      return nullptr;
    }
    auto c = std::shared_ptr<Cache>(dynamic_cast<Cache*>(e.release()));
    if (! c){
      cerr << __PRETTY_FUNCTION__ << endl;
      cerr << "fatal error in creating cache of type " << key.type() << endl;
      return nullptr;
    }
    c->_container = this;
    c->_parameters = key._parameters;
    insert(make_pair(key, c));
    c->update();
    return c;
  }

  OptimizableGraph::Vertex* CacheContainer::vertex() {
    return _vertex;
  }

  OptimizableGraph* CacheContainer::graph(){
    if (_vertex)
      return _vertex->graph();
    return nullptr;
  }

  void CacheContainer::update() {
    for (iterator it=begin(); it!=end(); ++it){
      (it->second)->update();
    }
    _updateNeeded=false;
  }

  void CacheContainer::setUpdateNeeded(bool needUpdate) {
    _updateNeeded=needUpdate;
    for (iterator it=begin(); it!=end(); ++it){
      (it->second)->_updateNeeded = needUpdate;
    }
  }

} // end namespace
