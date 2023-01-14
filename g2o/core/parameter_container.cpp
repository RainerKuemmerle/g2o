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

#include "parameter_container.h"

#include <iostream>

#include "factory.h"
#include "g2o/stuff/color_macros.h"
#include "g2o/stuff/macros.h"
#include "g2o/stuff/string_tools.h"
#include "parameter.h"

namespace g2o {

bool ParameterContainer::addParameter(std::shared_ptr<Parameter> p) {
  if (p->id() < 0) return false;
  auto it = find(p->id());
  if (it != end()) return false;
  emplace(p->id(), p);
  return true;
}

std::shared_ptr<Parameter> ParameterContainer::getParameter(int id) const {
  auto it = find(id);
  if (it == end()) return nullptr;
  return it->second;
}

std::shared_ptr<Parameter> ParameterContainer::detachParameter(int id) {
  auto it = find(id);
  if (it == end()) return nullptr;
  std::shared_ptr<Parameter> p = std::move(it->second);
  erase(it);
  return p;
}

bool ParameterContainer::write(std::ostream& os) const {
  Factory* factory = Factory::instance();
  for (const auto& it : *this) {
    os << factory->tag(it.second.get()) << " ";
    os << it.second->id() << " ";
    it.second->write(os);
    os << std::endl;
  }
  return true;
}

bool ParameterContainer::read(
    std::istream& is,
    const std::map<std::string, std::string>* renamedTypesLookup) {
  std::stringstream currentLine;
  std::string token;

  Factory* factory = Factory::instance();
  HyperGraph::GraphElemBitset elemBitset;
  elemBitset[HyperGraph::kHgetParameter] = true;

  while (true) {
    const int bytesRead = readLine(is, currentLine);
    if (bytesRead == -1) break;
    currentLine >> token;
    if (bytesRead == 0 || token.empty() || token[0] == '#') continue;
    if (renamedTypesLookup && !renamedTypesLookup->empty()) {
      auto foundIt = renamedTypesLookup->find(token);
      if (foundIt != renamedTypesLookup->end()) {
        token = foundIt->second;
      }
    }

    const std::shared_ptr<HyperGraph::HyperGraphElement> element =
        factory->construct(token, elemBitset);
    if (!element)  // not a parameter or otherwise unknown tag
      continue;
    assert(element->elementType() == HyperGraph::kHgetParameter &&
           "Should be a param");

    auto p = std::static_pointer_cast<Parameter>(element);
    int pid;
    currentLine >> pid;
    p->setId(pid);
    const bool r = p->read(currentLine);
    if (!r) {
      std::cerr << __PRETTY_FUNCTION__ << ": Error reading data " << token
                << " for parameter " << pid << std::endl;
    } else {
      if (!addParameter(p)) {
        std::cerr << __PRETTY_FUNCTION__ << ": Parameter of type:" << token
                  << " id:" << pid << " already defined" << std::endl;
      }
    }
  }  // while read line

  return true;
}

}  // namespace g2o
