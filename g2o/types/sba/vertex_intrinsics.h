// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#ifndef G2O_SBA_VERTEX_INTRINSICS_H
#define G2O_SBA_VERTEX_INTRINSICS_H

#include <Eigen/Core>
#include <iosfwd>

#include "g2o/core/base_vertex.h"
#include "g2o/core/eigen_types.h"
#include "g2o_types_sba_api.h"

namespace g2o {
template <typename T>
struct TypeTraits;

struct VertexIntrinsicsEstimate {
  VectorN<5> values;
};

/**
 * \brief Vertex encoding the intrinsics of the camera fx, fy, cx, xy, baseline;
 */
class G2O_TYPES_SBA_API VertexIntrinsics
    : public BaseVertex<4, VertexIntrinsicsEstimate> {
 public:
  VertexIntrinsics();
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void oplusImpl(const VectorX::MapType& update) override;
};

template <>
struct TypeTraits<VertexIntrinsicsEstimate> {
  enum {
    kVectorDimension = 5,
    kMinimalVectorDimension = 4,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = VertexIntrinsicsEstimate;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) { return t.values; }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    v = t.values;
  }

  static MinimalVectorType toMinimalVector(const Type& t) {
    return t.values.head<4>();
  }
  static void toMinimalData(const Type& t, double* data) {
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = t.values.head<4>();
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    Type res;
    res.values = v;
    return res;
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    Type res;
    res.values.head<4>() = v;
    res.values(4) = 0.1;
    return res;
  }

  static Type Identity() {
    Type res;
    res.values << 1., 1., .5, .5, .1;
    return res;
  }
};

}  // namespace g2o

#endif
