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

#include "property.h"

#include <vector>
#include <iostream>

#include "macros.h"

#include "string_tools.h"
using namespace std;

namespace g2o {

  BaseProperty::BaseProperty(const std::string name_) :_name(name_){
  }

  BaseProperty::~BaseProperty(){}

  bool PropertyMap::addProperty(BaseProperty* p) {
    std::pair<PropertyMapIterator,bool> result = insert(make_pair(p->name(), p));
    return result.second;
  }

  bool PropertyMap::eraseProperty(const std::string& name) {
    PropertyMapIterator it=find(name);
    if (it==end())
      return false;
    delete it->second;
    erase(it);
    return true;
  }

  PropertyMap::~PropertyMap() {
    for (PropertyMapIterator it=begin(); it!=end(); it++){
      if (it->second)
        delete it->second;
    }
  }

  bool PropertyMap::updatePropertyFromString(const std::string& name, const std::string& value)
  {
    PropertyMapIterator it = find(name);
    if (it == end())
      return false;
    it->second->fromString(value);
    return true;
  }

  bool PropertyMap::updateMapFromString(const std::string& values)
  {
    bool status = true;
    vector<string> valuesMap = strSplit(values, ",");
    for (size_t i = 0; i < valuesMap.size(); ++i) {
      vector<string> m = strSplit(valuesMap[i], "=");
      if (m.size() != 2) {
        cerr << __PRETTY_FUNCTION__ << ": unable to extract name=value pair from " << valuesMap[i] << endl;
        continue;
      }
      string name = trim(m[0]);
      string value = trim(m[1]);
      status = status && updatePropertyFromString(name, value);
    }
    return status;
  }

} // end namespace
