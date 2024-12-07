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

#ifndef G2O_LINE3D_H_
#define G2O_LINE3D_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "g2o_types_slam3d_addons_api.h"

namespace g2o {

using Matrix6x4 = Eigen::Matrix<double, 6, 4>;

struct OrthonormalLine3D {
  Matrix2 W;
  Matrix3 U;

  OrthonormalLine3D() {
    W = Matrix2::Identity();
    U = Matrix3::Identity();
  }
};

class G2O_TYPES_SLAM3D_ADDONS_API Line3D {
 public:
  Vector6 line;

  Line3D();

  explicit Line3D(const Vector6& v);

  [[nodiscard]] Vector6 toCartesian() const;
  static Line3D fromCartesian(const Vector6& cart);

  [[nodiscard]] Vector3 w() const { return line.head<3>(); }
  void setW(const Vector3& w_);

  [[nodiscard]] Vector3 d() const { return line.tail<3>(); }
  void setD(const Vector3& d_);

  [[nodiscard]] OrthonormalLine3D toOrthonormal() const;
  static Line3D fromOrthonormal(const OrthonormalLine3D& ortho);

  void normalize();
  [[nodiscard]] Line3D normalized() const;

  void oplus(const Vector4& v);
  [[nodiscard]] Vector4 ominus(const Line3D& line) const;
};

G2O_TYPES_SLAM3D_ADDONS_API Line3D operator*(const Isometry3& t,
                                             const Line3D& line);

namespace internal {

G2O_TYPES_SLAM3D_ADDONS_API Vector6 transformCartesianLine(const Isometry3& t,
                                                           const Vector6& line);

G2O_TYPES_SLAM3D_ADDONS_API Vector6 normalizeCartesianLine(const Vector6& line);

G2O_TYPES_SLAM3D_ADDONS_API double mline_elevation(const double v[3]);

G2O_TYPES_SLAM3D_ADDONS_API double getAzimuth(const Vector3& direction);

G2O_TYPES_SLAM3D_ADDONS_API double getElevation(const Vector3& direction);

}  // namespace internal

template <>
struct TypeTraits<Line3D> {
  enum {  // NOLINT
    kVectorDimension = 6,
    kMinimalVectorDimension = 6,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = Line3D;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) { return t.line; }
  static void toData(const Type& t, double* data) {  // NOLINT
    typename VectorType::MapType v(data, kVectorDimension);
    v = t.line;
  }

  static MinimalVectorType toMinimalVector(const Type& t) { return t.line; }
  static void toMinimalData(const Type& t, double* data) {  // NOLINT
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = t.line;
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    return Line3D(v);
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return Line3D(v);
  }

  static Type Identity() { return Type(); }
};

}  // namespace g2o

#endif
