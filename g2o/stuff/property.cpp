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

#include "property.h"

#include <iostream>
#include <vector>

#include "macros.h"
#include "string_tools.h"

namespace g2o {

BaseProperty::BaseProperty(std::string name) : name_(std::move(name)) {}

bool PropertyMap::addProperty(const std::shared_ptr<BaseProperty>& p) {
  const std::pair<PropertyMapIterator, bool> result =
      insert(make_pair(p->name(), p));
  return result.second;
}

bool PropertyMap::eraseProperty(const std::string& name) {
  auto it = find(name);
  if (it == end()) return false;
  erase(it);
  return true;
}

bool PropertyMap::updatePropertyFromString(
    const std::string& name, const std::string& value) {  // NOLINT
  auto it = find(name);
  if (it == end()) return false;
  it->second->fromString(value);
  return true;
}

void PropertyMap::writeToCSV(std::ostream& os) const {
  for (auto it = cbegin(); it != cend(); ++it) {
    if (it != cbegin()) os << ",";
    os << it->second->name();
  }
  os << std::endl;
  for (auto it = cbegin(); it != cend(); ++it) {
    if (it != cbegin()) os << ",";
    os << it->second->toString();
  }
  os << std::endl;
}

bool PropertyMap::updateMapFromString(const std::string& values) {
  bool status = true;
  const std::vector<std::string> valuesMap = strSplit(values, ",");
  for (const auto& entry : valuesMap) {
    std::vector<std::string> m = strSplit(entry, "=");
    if (m.size() != 2) {
      std::cerr << __PRETTY_FUNCTION__
                << ": unable to extract name=value pair from " << entry
                << std::endl;
      continue;
    }
    const std::string name = trim(m[0]);
    const std::string value = trim(m[1]);
    status = status && updatePropertyFromString(name, value);
  }
  return status;
}

}  // namespace g2o
