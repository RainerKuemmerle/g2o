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

#ifndef G2O_LINE2D_H
#define G2O_LINE2D_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o_types_slam2d_addons_api.h"

namespace g2o {

class G2O_TYPES_SLAM2D_ADDONS_API Line2D {
 public:
  Line2D();
  explicit Line2D(const Vector2& v);

  void setZero();

  const double& operator[](int idx) const { return values_[idx]; }
  double& operator[](int idx) { return values_[idx]; }

 protected:
  double values_[2];
};

G2O_TYPES_SLAM2D_ADDONS_API Line2D operator*(const SE2& t, const Line2D& l);

template <>
struct TypeTraits<Line2D> {
  enum {
    kVectorDimension = 2,
    kMinimalVectorDimension = 2,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = Line2D;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) {
    VectorType result;
    result(0) = t[0];
    result(1) = t[1];
    return result;
  }
  static void toData(const Type& t, double* data) {
    data[0] = t[0];
    data[1] = t[1];
  }

  static MinimalVectorType toMinimalVector(const Type& t) {
    return TypeTraits<Line2D>::toVector(t);
  }
  static void toMinimalData(const Type& t, double* data) {
    TypeTraits<Line2D>::toData(t, data);
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    return Line2D(v);
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return Line2D(v);
  }

  static Type Identity() { return Type(); }
};

}  // namespace g2o

#endif
