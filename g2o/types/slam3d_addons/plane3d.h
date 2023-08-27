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

#ifndef G2O_PLANE3D_H_
#define G2O_PLANE3D_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "g2o/stuff/misc.h"
#include "g2o_types_slam3d_addons_api.h"

namespace g2o {

class G2O_TYPES_SLAM3D_ADDONS_API Plane3D {
 public:
  friend Plane3D operator*(const Isometry3& t, const Plane3D& plane);

  Plane3D() { fromVector(Vector4(1., 0., 0., -1.)); }

  explicit Plane3D(const Vector4& v) { fromVector(v); }

  [[nodiscard]] inline Vector4 toVector() const { return coeffs_; }

  [[nodiscard]] inline const Vector4& coeffs() const { return coeffs_; }

  inline void fromVector(const Vector4& coeffs) {
    coeffs_ = coeffs;
    normalize(coeffs_);
  }

  static double azimuth(const Vector3& v) { return std::atan2(v(1), v(0)); }

  static double elevation(const Vector3& v) {
    return std::atan2(v(2), v.head<2>().norm());
  }

  [[nodiscard]] double distance() const { return -coeffs_(3); }

  [[nodiscard]] Vector3 normal() const { return coeffs_.head<3>(); }

  static Matrix3 rotation(const Vector3& v) {
    double _azimuth = azimuth(v);
    double _elevation = elevation(v);
    return (AngleAxis(_azimuth, Vector3::UnitZ()) *
            AngleAxis(-_elevation, Vector3::UnitY()))
        .toRotationMatrix();
  }

  inline void oplus(const Vector3& v) {
    // construct a normal from azimuth and elevation;
    double _azimuth = v[0];
    double _elevation = v[1];
    double s = std::sin(_elevation);
    double c = std::cos(_elevation);
    Vector3 n(c * std::cos(_azimuth), c * std::sin(_azimuth), s);

    // rotate the normal
    Matrix3 R = rotation(normal());
    double d = distance() + v[2];
    coeffs_.head<3>() = R * n;
    coeffs_(3) = -d;
    normalize(coeffs_);
  }

  [[nodiscard]] inline Vector3 ominus(const Plane3D& plane) const {
    // construct the rotation that would bring the plane normal in (1 0 0)
    Matrix3 R = rotation(normal()).transpose();
    Vector3 n = R * plane.normal();
    double d = distance() - plane.distance();
    return Vector3(azimuth(n), elevation(n), d);
  }

 protected:
  static inline void normalize(Vector4& coeffs) {
    double n = coeffs.head<3>().norm();
    coeffs = coeffs * (1. / n);
  }

  Vector4 coeffs_;
};

inline Plane3D operator*(const Isometry3& t, const Plane3D& plane) {
  Vector4 v = plane.coeffs_;
  Vector4 v2;
  Matrix3 R = t.rotation();
  v2.head<3>() = R * v.head<3>();
  v2(3) = v(3) - t.translation().dot(v2.head<3>());
  return Plane3D(v2);
};

/**
 * @brief TypeTraits specialization for a Plane3D
 */
template <>
struct TypeTraits<Plane3D> {
  enum {
    kVectorDimension = 4,
    kMinimalVectorDimension = 4,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = Plane3D;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) { return t.coeffs(); }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    v = t.coeffs();
  }

  static MinimalVectorType toMinimalVector(const Type& t) { return t.coeffs(); }
  static void toMinimalData(const Type& t, double* data) {
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = t.coeffs();
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    return Plane3D(v);
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return Plane3D(v);
  }

  static Type Identity() { return Plane3D(); }
};

}  // namespace g2o

#endif
