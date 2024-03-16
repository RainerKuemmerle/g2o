// g2o - General Graph Optimization
// Copyright (C) 2023 R. Kuemmerle
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

#ifndef TYPE_TRAITS_ISOMETRY_H
#define TYPE_TRAITS_ISOMETRY_H

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "isometry3d_mappings.h"

namespace g2o {

/**
 * @brief TypeTraits specialization for an Isometry3
 */
template <>
struct TypeTraits<Isometry3> {
  enum {
    kVectorDimension = 7,
    kMinimalVectorDimension = 6,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = Isometry3;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) { return internal::toVectorQT(t); }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    v = toVector(t);
  }

  static MinimalVectorType toMinimalVector(const Type& t) {
    return internal::toVectorMQT(t);
  }
  static void toMinimalData(const Type& t, double* data) {
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = toMinimalVector(t);
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    return internal::fromVectorQT(v);
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return internal::fromVectorMQT(v);
  }
};

}  // namespace g2o

#endif
