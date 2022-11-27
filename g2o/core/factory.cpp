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

namespace g2o {

using std::cerr;
using std::endl;

std::unique_ptr<Factory> Factory::factoryInstance_;

Factory* Factory::instance() {
  if (factoryInstance_ == nullptr) {
    factoryInstance_.reset(new Factory);
#ifdef G2O_DEBUG_FACTORY
    cerr << "# Factory allocated " << (void*)factoryInstance.get() << endl;
#endif
  }

  return factoryInstance_.get();
}

void Factory::registerType(
    const std::string& tag,
    std::unique_ptr<AbstractHyperGraphElementCreator> c) {
  const CreatorMap::const_iterator foundIt = creator_.find(tag);
  if (foundIt != creator_.end()) {
    cerr << "FACTORY WARNING: Overwriting Vertex tag " << tag << endl;
    assert(0);
  }
  const TagLookup::const_iterator tagIt = tagLookup_.find(c->name());
  if (tagIt != tagLookup_.end()) {
    cerr << "FACTORY WARNING: Registering same class for two tags " << c->name()
         << endl;
    assert(0);
  }

#ifdef G2O_DEBUG_FACTORY
  cerr << "# Factory " << (void*)this << " constructing type " << tag << " ";
#endif
  // construct an element once to figure out its type
  std::unique_ptr<HyperGraph::HyperGraphElement> element = c->construct();

#ifdef G2O_DEBUG_FACTORY
  cerr << "done." << endl;
  cerr << "# Factory " << (void*)this << " registering " << tag;
  cerr << " " << (void*)c.get() << " ";
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
      assert(0 && "Unknown element type occurred, fix elementTypes");
      break;
  }
  cerr << endl;
#endif

  std::unique_ptr<CreatorInformation> ci =
      std::make_unique<CreatorInformation>();
  ci->elementTypeBit = element->elementType();
  ci->creator = std::move(c);
  tagLookup_[ci->creator->name()] = tag;
  creator_[tag] = std::move(ci);
}

void Factory::unregisterType(const std::string& tag) {
  // Look for the tag
  auto tagPosition = creator_.find(tag);

  if (tagPosition != creator_.end()) {
    const auto& c = tagPosition->second->creator;

    // If we found it, remove the creator from the tag lookup map
    auto classPosition = tagLookup_.find(c->name());
    if (classPosition != tagLookup_.end()) {
      tagLookup_.erase(classPosition);
    }
    creator_.erase(tagPosition);
  }
}

std::unique_ptr<HyperGraph::HyperGraphElement> Factory::construct(
    const std::string& tag) const {
  auto foundIt = creator_.find(tag);
  if (foundIt != creator_.end()) {
    // cerr << "tag " << tag << " -> " << (void*) foundIt->second->creator << "
    // " << foundIt->second->creator->name() << endl;
    return foundIt->second->creator->construct();
  }
  return nullptr;
}

const std::string& Factory::tag(const HyperGraph::HyperGraphElement* e) const {
  static const std::string kEmptyStr;
  auto foundIt = tagLookup_.find(typeid(*e).name());
  if (foundIt != tagLookup_.end()) return foundIt->second;
  return kEmptyStr;
}

void Factory::fillKnownTypes(std::vector<std::string>& types) const {
  types.clear();
  for (const auto& it : creator_) types.push_back(it.first);
}

bool Factory::knowsTag(const std::string& tag, int* elementType) const {
  auto foundIt = creator_.find(tag);
  if (foundIt == creator_.end()) {
    if (elementType) *elementType = -1;
    return false;
  }
  if (elementType) *elementType = foundIt->second->elementTypeBit;
  return true;
}

void Factory::destroy() {
  std::unique_ptr<Factory> aux;
  factoryInstance_.swap(aux);
}

void Factory::printRegisteredTypes(std::ostream& os, bool comment) const {
  if (comment) os << "# ";
  os << "types:" << endl;
  for (const auto& it : creator_) {
    if (comment) os << "#";
    cerr << "\t" << it.first << endl;
  }
}

std::unique_ptr<HyperGraph::HyperGraphElement> Factory::construct(
    const std::string& tag,
    const HyperGraph::GraphElemBitset& elemsToConstruct) const {
  if (elemsToConstruct.none()) {
    return construct(tag);
  }
  auto foundIt = creator_.find(tag);
  if (foundIt != creator_.end() && foundIt->second->elementTypeBit >= 0 &&
      elemsToConstruct.test(foundIt->second->elementTypeBit)) {
    return foundIt->second->creator->construct();
  }
  return nullptr;
}

}  // namespace g2o
