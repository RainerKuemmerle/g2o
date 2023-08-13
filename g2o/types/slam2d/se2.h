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

#ifndef G2O_SE2_H_
#define G2O_SE2_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "g2o/stuff/misc.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

/**
 * \brief represent SE2
 */
class G2O_TYPES_SLAM2D_API SE2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SE2() : R_(0), t_(0, 0) {}

  explicit SE2(const Isometry2& iso) : R_(0), t_(iso.translation()) {
    R_.fromRotationMatrix(iso.linear());
  }

  explicit SE2(const Vector3& v) : R_(v[2]), t_(v[0], v[1]) {}

  SE2(double x, double y, double theta) : R_(theta), t_(x, y) {}

  //! translational component
  [[nodiscard]] const Vector2& translation() const { return t_; }
  void setTranslation(const Vector2& t) { t_ = t; }

  //! rotational component
  [[nodiscard]] const Rotation2D& rotation() const { return R_; }
  void setRotation(const Rotation2D& R) { R_ = R; }

  //! concatenate two SE2 elements (motion composition)
  SE2 operator*(const SE2& tr2) const {
    SE2 result(*this);
    result *= tr2;
    return result;
  }

  //! motion composition operator
  SE2& operator*=(const SE2& tr2) {
    t_ += R_ * tr2.t_;
    R_.angle() += tr2.R_.angle();
    R_.angle() = normalize_theta(R_.angle());
    return *this;
  }

  //! project a 2D vector
  Vector2 operator*(const Vector2& v) const { return t_ + R_ * v; }

  //! invert :-)
  [[nodiscard]] SE2 inverse() const {
    SE2 ret;
    ret.R_ = R_.inverse();
    ret.R_.angle() = normalize_theta(ret.R_.angle());
#ifdef _MSC_VER
    ret.t_ = ret.R_ * (Vector2(t_ * -1.));
#else
    ret.t_ = ret.R_ * (t_ * -1.);
#endif
    return ret;
  }

  inline double operator[](int i) const {
    assert(i >= 0 && i < 3);
    if (i < 2) return t_(i);
    return R_.angle();
  }

  //! assign from a 3D vector (x, y, theta)
  inline void fromVector(const Vector3& v) { *this = SE2(v[0], v[1], v[2]); }

  //! convert to a 3D vector (x, y, theta)
  [[nodiscard]] Vector3 toVector() const {
    return Vector3(t_.x(), t_.y(), R_.angle());
  }

  [[nodiscard]] Isometry2 toIsometry() const {
    Isometry2 iso = Isometry2::Identity();
    iso.linear() = R_.toRotationMatrix();
    iso.translation() = t_;
    return iso;
  }

 protected:
  Rotation2D R_;
  Vector2 t_;
};

template <>
struct TypeTraits<SE2> {
  enum {
    kVectorDimension = 3,
    kMinimalVectorDimension = 3,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = SE2;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) { return t.toVector(); }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    v = t.toVector();
  }

  static MinimalVectorType toMinimalVector(const Type& t) {
    return t.toVector();
  }
  static void toMinimalData(const Type& t, double* data) {
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = t.toVector();
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    return SE2(v[0], v[1], v[2]);
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    return SE2(v[0], v[1], v[2]);
  }

  static Type Identity() { return SE2(); }
};

}  // namespace g2o

#endif
