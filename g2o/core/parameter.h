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

#ifndef G2O_GRAPH_PARAMETER_HH_
#define G2O_GRAPH_PARAMETER_HH_

#include <iosfwd>
#include <memory>
#include <vector>

#include "g2o/core/g2o_core_api.h"
#include "g2o/core/type_traits.h"
#include "hyper_graph.h"

namespace g2o {

class G2O_CORE_API Parameter : public HyperGraph::HyperGraphElement {
 public:
  Parameter() = default;
  ~Parameter() override = default;
  //! read the data from a stream
  virtual bool read(std::istream& is) = 0;
  //! write the data to a stream
  virtual bool write(std::ostream& os) const = 0;
  [[nodiscard]] int id() const { return id_; }
  void setId(int id_);
  [[nodiscard]] HyperGraph::HyperGraphElementType elementType() const final {
    return HyperGraph::kHgetParameter;
  }

  [[nodiscard]] virtual int parameterDimension() const = 0;
  [[nodiscard]] virtual int minimalParameterDimension() const = 0;

 protected:
  int id_ = -1;
};

template <typename T>
class BaseParameter : public Parameter {
 public:
  using ParameterType = T;

  [[nodiscard]] int parameterDimension() const final {
    static_assert(TypeTraits<ParameterType>::kMinimalVectorDimension != INT_MIN,
                  "Forgot to implement TypeTrait for your Estimate");
    return TypeTraits<ParameterType>::kVectorDimension;
  }

  [[nodiscard]] int minimalParameterDimension() const final {
    static_assert(TypeTraits<ParameterType>::kMinimalVectorDimension != INT_MIN,
                  "Forgot to implement TypeTrait for your Estimate");
    return TypeTraits<ParameterType>::kMinimalVectorDimension;
  }

  const ParameterType& param() const { return parameter_; }
  ParameterType& param() { return parameter_; }

 protected:
  ParameterType parameter_;
};

using ParameterVector = std::vector<std::shared_ptr<Parameter>>;

}  // namespace g2o

#endif
