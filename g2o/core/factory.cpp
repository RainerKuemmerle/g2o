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

#include "factory.h"

#include <cassert>
#include <iostream>
#include <typeinfo>

#include "cache.h"
#include "creators.h"
#include "g2o/stuff/color_macros.h"
#include "optimizable_graph.h"
#include "parameter.h"
#include "graph.pb.h"

using namespace std;

namespace g2o {

std::unique_ptr<Factory> Factory::factoryInstance;

Factory* Factory::instance() {
  if (factoryInstance.get() == nullptr) {
    factoryInstance.reset(new Factory);
#ifdef G2O_DEBUG_FACTORY
    cerr << "# Factory allocated " << (void*)factoryInstance.get() << endl;
#endif
  }

  return factoryInstance.get();
}

void Factory::registerType(const std::string& tag,
                           const std::shared_ptr<AbstractHyperGraphElementCreator>& c) {
  CreatorMap::const_iterator foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    cerr << "FACTORY WARNING: Overwriting Vertex tag " << tag << endl;
    assert(0);
  }
  TagLookup::const_iterator tagIt = _tagLookup.find(c->name());
  if (tagIt != _tagLookup.end()) {
    cerr << "FACTORY WARNING: Registering same class for two tags " << c->name() << endl;
    assert(0);
  }

  CreatorInformation* ci = new CreatorInformation();
  ci->creator = c;

#ifdef G2O_DEBUG_FACTORY
  cerr << "# Factory " << (void*)this << " constructing type " << tag << " ";
#endif
  // construct an element once to figure out its type
  HyperGraph::HyperGraphElement* element = c->construct();
  ci->elementTypeBit = element->elementType();

#ifdef G2O_DEBUG_FACTORY
  cerr << "done." << endl;
  cerr << "# Factory " << (void*)this << " registering " << tag;
  cerr << " " << (void*)c << " ";
  switch (element->elementType()) {
    case HyperGraph::HGET_VERTEX:
      cerr << " -> Vertex";
      break;
    case HyperGraph::HGET_EDGE:
      cerr << " -> Edge";
      break;
    case HyperGraph::HGET_PARAMETER:
      cerr << " -> Parameter";
      break;
    case HyperGraph::HGET_CACHE:
      cerr << " -> Cache";
      break;
    case HyperGraph::HGET_DATA:
      cerr << " -> Data";
      break;
    default:
      assert(0 && "Unknown element type occured, fix elementTypes");
      break;
  }
  cerr << endl;
#endif

  _creator[tag] = std::unique_ptr<CreatorInformation>(ci);
  _tagLookup[c->name()] = tag;
  delete element;
}

void Factory::unregisterType(const std::string& tag) {
  // Look for the tag
  CreatorMap::iterator tagPosition = _creator.find(tag);

  if (tagPosition != _creator.end()) {
    const auto& c = tagPosition->second->creator;

    // If we found it, remove the creator from the tag lookup map
    TagLookup::iterator classPosition = _tagLookup.find(c->name());
    if (classPosition != _tagLookup.end()) {
      _tagLookup.erase(classPosition);
    }
    _creator.erase(tagPosition);
  }
}

HyperGraph::HyperGraphElement* Factory::construct(const std::string& tag) const {
  CreatorMap::const_iterator foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    // cerr << "tag " << tag << " -> " << (void*) foundIt->second->creator << " " <<
    // foundIt->second->creator->name() << endl;
    return foundIt->second->creator->construct();
  }
  return nullptr;
}

const std::string& Factory::tag(const HyperGraph::HyperGraphElement* e) const {
  static std::string emptyStr("");
  TagLookup::const_iterator foundIt = _tagLookup.find(typeid(*e).name());
  if (foundIt != _tagLookup.end()) return foundIt->second;
  return emptyStr;
}

void Factory::fillKnownTypes(std::vector<std::string>& types) const {
  types.clear();
  for (CreatorMap::const_iterator it = _creator.begin(); it != _creator.end(); ++it)
    types.push_back(it->first);
}

bool Factory::knowsTag(const std::string& tag, int* elementType) const {
  CreatorMap::const_iterator foundIt = _creator.find(tag);
  if (foundIt == _creator.end()) {
    if (elementType) *elementType = -1;
    return false;
  }
  if (elementType) *elementType = foundIt->second->elementTypeBit;
  return true;
}

void Factory::destroy() {
  std::unique_ptr<Factory> aux;
  factoryInstance.swap(aux);
}

g2o::proto::FactorTags Factory::strTagToEnum(const std::string& tag) {
  if (TOKEN_VERTEX_SE3_QUAT.compare(tag) == 0) {
    return g2o::proto::FactorTags::VERTEX_SE3_QUAT;
  } else if (TOKEN_EDGE_SE3_PRIOR.compare(tag) == 0) {
    return g2o::proto::FactorTags::EDGE_SE3_PRIOR;
  } else if (TOKEN_EDGE_SE3_QUAT.compare(tag) == 0) {
    return g2o::proto::FactorTags::EDGE_SE3_QUAT;
  } else if (TOKEN_PARAMS_SE3_OFFSET.compare(tag) == 0) {
    return g2o::proto::FactorTags::PARAMS_SE3_OFFSET;
  } else {
    cerr << "Unknown tag " << tag << endl;
    return g2o::proto::FactorTags::UNKNOWN;
  }
}

const std::string& Factory::enumToStrTag(const g2o::proto::FactorTags tag) {
  if (g2o::proto::FactorTags::VERTEX_SE3_QUAT == tag) {
    return TOKEN_VERTEX_SE3_QUAT;
  } else if (g2o::proto::FactorTags::EDGE_SE3_PRIOR == tag) {
    return TOKEN_EDGE_SE3_PRIOR;
  } else if (g2o::proto::FactorTags::EDGE_SE3_QUAT == tag) {
    return TOKEN_EDGE_SE3_QUAT;
  } else if (g2o::proto::FactorTags::PARAMS_SE3_OFFSET == tag) {
    return TOKEN_PARAMS_SE3_OFFSET;
  } else {
    cerr << "Unknown tag " << tag << endl;
    return TOKEN_UNKNOWN;
  }
}

void Factory::printRegisteredTypes(std::ostream& os, bool comment) const {
  if (comment) os << "# ";
  os << "types:" << endl;
  for (CreatorMap::const_iterator it = _creator.begin(); it != _creator.end(); ++it) {
    if (comment) os << "#";
    cerr << "\t" << it->first << endl;
  }
}

HyperGraph::HyperGraphElement* Factory::construct(
    const std::string& tag, const HyperGraph::GraphElemBitset& elemsToConstruct) const {
  if (elemsToConstruct.none()) {
    return construct(tag);
  }
  CreatorMap::const_iterator foundIt = _creator.find(tag);
  if (foundIt != _creator.end() && foundIt->second->elementTypeBit >= 0 &&
      elemsToConstruct.test(foundIt->second->elementTypeBit)) {
    return foundIt->second->creator->construct();
  }
  return nullptr;
}

}  // namespace g2o
