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

#ifndef G2O_PROPERTY_H_
#define G2O_PROPERTY_H_

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "g2o_stuff_api.h"
#include "string_tools.h"

namespace g2o {

class G2O_STUFF_API BaseProperty {
 public:
  explicit BaseProperty(std::string name_);
  virtual ~BaseProperty() = default;
  const std::string& name() { return name_; }
  virtual std::string toString() const = 0;
  virtual bool fromString(const std::string& s) = 0;

 protected:
  std::string name_;
};

template <typename T>
class Property : public BaseProperty {
 public:
  using ValueType = T;
  explicit Property(const std::string& name_) : BaseProperty(name_) {}
  Property(const std::string& name_, T v)
      : BaseProperty(name_), value_(std::move(v)) {}
  void setValue(const T& v) { value_ = v; }
  const T& value() const { return value_; }
  std::string toString() const override {
    std::stringstream sstr;
    sstr << value_;
    return sstr.str();
  }
  bool fromString(const std::string& s) override {
    bool status = convertString(s, value_);
    return status;
  }

 protected:
  T value_;
};

/**
 * \brief a collection of properties mapping from name to the property itself
 */
class G2O_STUFF_API PropertyMap
    : protected std::map<std::string, std::shared_ptr<BaseProperty>> {
 public:
  using BaseClass = std::map<std::string, std::shared_ptr<BaseProperty>>;
  using PropertyMapIterator = BaseClass::iterator;
  using PropertyMapConstIterator = BaseClass::const_iterator;

  ~PropertyMap() = default;

  /**
   * add a property to the map
   */
  bool addProperty(const std::shared_ptr<BaseProperty>& p);

  /**
   * remove a property from the map
   */
  bool eraseProperty(const std::string& name_);

  /**
   * return a property by its name
   */
  template <typename P>
  std::shared_ptr<P> getProperty(const std::string& name) {
    auto it = find(name);
    if (it == end()) return nullptr;
    return std::dynamic_pointer_cast<P>(it->second);
  }
  template <typename P>
  std::shared_ptr<const P> getProperty(const std::string& name) const {
    auto it = find(name);
    if (it == end()) return nullptr;
    return std::dynamic_pointer_cast<const P>(it->second);
  }

  /**
   * create a property and insert it
   */
  template <typename P>
  std::shared_ptr<P> makeProperty(const std::string& name_,
                                  const typename P::ValueType& v) {
    auto it = find(name_);
    if (it == end()) {
      std::shared_ptr<P> p = std::make_shared<P>(name_, v);
      addProperty(p);
      return p;
    }
    return std::dynamic_pointer_cast<P>(it->second);
  }

  /**
   * update a specific property with a new value
   * @return true if the params is stored and update was carried out
   */
  bool updatePropertyFromString(const std::string& name,
                                const std::string& value);

  /**
   * update the map based on a name=value string, e.g., name1=val1,name2=val2
   * @return true, if it was possible to update all parameters
   */
  bool updateMapFromString(const std::string& values);

  void writeToCSV(std::ostream& os) const;

  using BaseClass::begin;
  using BaseClass::const_iterator;
  using BaseClass::end;
  using BaseClass::iterator;
  using BaseClass::size;
};

using IntProperty = Property<int>;
using BoolProperty = Property<bool>;
using FloatProperty = Property<float>;
using DoubleProperty = Property<double>;
using StringProperty = Property<std::string>;

}  // namespace g2o
#endif
