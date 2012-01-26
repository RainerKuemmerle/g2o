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

#include "factory.h"

#include "creators.h"
#include "parameter.h"
#include "cache.h"
#include "optimizable_graph.h"
#include "g2o/stuff/color_macros.h"

#include <iostream>
#include <typeinfo>
#include <cassert>

// define to get some verbose output
#define DEBUG_FACTORY

using namespace std;

namespace g2o {

Factory* Factory::factoryInstance = 0;

Factory::Factory()
{
}

Factory::~Factory()
{
# ifdef DEBUG_FACTORY
  cerr << "# Factory destroying " << (void*)this << endl;
# endif
  for (CreatorMap::iterator it = _creator.begin(); it != _creator.end(); ++it) {
    delete it->second->creator;
  }
  _creator.clear();
  _tagLookup.clear();
}

Factory* Factory::instance()
{
  if (factoryInstance == 0) {
    factoryInstance = new Factory;
#  ifdef DEBUG_FACTORY
    cerr << "# Factory allocated " << (void*)factoryInstance << endl;
#  endif
  }

  return factoryInstance;
}

void Factory::registerType(const std::string& tag, AbstractHyperGraphElementCreator* c)
{
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

  // construct an element once to figur out its type
  HyperGraph::HyperGraphElement* element = c->construct();
  ci->elementTypeBit = element->elementType();

#ifdef DEBUG_FACTORY
  cerr << "# Factory " << (void*)this << " registering " << tag;
  cerr << " " << (void*) c << " ";
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

  _creator[tag] = ci;
  _tagLookup[c->name()] = tag;
  delete element;
}

  void Factory::unregisterType(const std::string& tag)
  {
    // Look for the tag
    CreatorMap::iterator tagPosition = _creator.find(tag);

    if (tagPosition != _creator.end()) {

      AbstractHyperGraphElementCreator* c = tagPosition->second->creator;

      // If we found it, remove the creator from the tag lookup map
      TagLookup::iterator classPosition = _tagLookup.find(c->name());
      if (classPosition != _tagLookup.end())
        {
          _tagLookup.erase(classPosition);
        }
      _creator.erase(tagPosition);
    }
  }

HyperGraph::HyperGraphElement* Factory::construct(const std::string& tag) const
{
  CreatorMap::const_iterator foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    //cerr << "tag " << tag << " -> " << (void*) foundIt->second->creator << " " << foundIt->second->creator->name() << endl;
    return foundIt->second->creator->construct();
  }
  return 0;
}

const std::string& Factory::tag(const HyperGraph::HyperGraphElement* e) const
{
  static std::string emptyStr("");
  TagLookup::const_iterator foundIt = _tagLookup.find(typeid(*e).name());
  if (foundIt != _tagLookup.end())
    return foundIt->second;
  return emptyStr;
}

void Factory::fillKnownTypes(std::vector<std::string>& types) const
{
  types.clear();
  for (CreatorMap::const_iterator it = _creator.begin(); it != _creator.end(); ++it)
    types.push_back(it->first);
}

bool Factory::knowsTag(const std::string& tag, int* elementType) const
{
  CreatorMap::const_iterator foundIt = _creator.find(tag);
  if (foundIt == _creator.end()) {
    if (elementType)
      *elementType = -1;
    return false;
  }
  if (elementType)
    *elementType = foundIt->second->elementTypeBit;
  return true;
}

void Factory::destroy()
{
  delete factoryInstance;
  factoryInstance = 0;
}

void Factory::printRegisteredTypes(std::ostream& os, bool comment) const
{
  if (comment)
    os << "# ";
  os << "types:" << endl;
  for (CreatorMap::const_iterator it = _creator.begin(); it != _creator.end(); ++it) {
    if (comment)
      os << "#";
    cerr << "\t" << it->first << endl;
  }
}

HyperGraph::HyperGraphElement* Factory::construct(const std::string& tag, const HyperGraph::GraphElemBitset& elemsToConstruct) const
{
  if (elemsToConstruct.none()) {
    return construct(tag);
  }
  CreatorMap::const_iterator foundIt = _creator.find(tag);
  if (foundIt != _creator.end() && foundIt->second->elementTypeBit >= 0 && elemsToConstruct.test(foundIt->second->elementTypeBit)) {
    return foundIt->second->creator->construct();
  }
  return 0;
}

} // end namespace

