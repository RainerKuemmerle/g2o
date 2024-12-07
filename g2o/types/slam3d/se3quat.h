// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_SE3QUAT_H_
#define G2O_SE3QUAT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>

#include "g2o/core/eigen_types.h"
#include "g2o/core/type_traits.h"
#include "g2o/stuff/misc.h"
#include "g2o_types_slam3d_api.h"

namespace g2o {

class G2O_TYPES_SLAM3D_API SE3Quat {
 protected:
  Quaternion r_;
  Vector3 t_;

 public:
  SE3Quat();
  SE3Quat(const Matrix3& R, Vector3 t);
  SE3Quat(const Quaternion& q, const Vector3& t);

  /**
   * templatized constructor which allows v to be an arbitrary Eigen Vector
   * type, e.g., Vector6 or Map<Vector6>
   */
  template <typename Derived>
  explicit SE3Quat(const Eigen::MatrixBase<Derived>& v) {
    assert((v.size() == 6 || v.size() == 7) &&
           "Vector dimension does not match");
    if (v.size() == 6) {
      for (int i = 0; i < 3; i++) {
        t_[i] = v[i];
        r_.coeffs()(i) = v[i + 3];
      }
      r_.w() = 0.;  // recover the positive w
      if (r_.norm() > 1.) {
        r_.normalize();
      } else {
        double w2 = cst(1.) - r_.squaredNorm();
        r_.w() = (w2 < cst(0.)) ? cst(0.) : std::sqrt(w2);
      }
    } else if (v.size() == 7) {
      int idx = 0;
      for (int i = 0; i < 3; ++i, ++idx) t_(i) = v(idx);
      for (int i = 0; i < 4; ++i, ++idx) r_.coeffs()(i) = v(idx);
      normalizeRotation();
    }
  }

  [[nodiscard]] const Vector3& translation() const { return t_; }
  void setTranslation(const Vector3& t);

  [[nodiscard]] const Quaternion& rotation() const { return r_; }
  void setRotation(const Quaternion& r);

  SE3Quat operator*(const SE3Quat& tr2) const;

  SE3Quat& operator*=(const SE3Quat& tr2);

  Vector3 operator*(const Vector3& v) const;

  [[nodiscard]] SE3Quat inverse() const;

  double operator[](int i) const {
    assert(i < 7);
    if (i < 3) return t_[i];
    return r_.coeffs()[i - 3];
  }

  [[nodiscard]] Vector7 toVector() const;

  void fromVector(const Vector7& v);

  [[nodiscard]] Vector6 toMinimalVector() const;

  void fromMinimalVector(const Vector6& v);

  [[nodiscard]] Vector6 log() const;

  [[nodiscard]] Vector3 map(const Vector3& xyz) const;

  static SE3Quat exp(const Vector6& update);

  [[nodiscard]] Matrix6 adj() const;

  [[nodiscard]] MatrixN<4> to_homogeneous_matrix() const;

  void normalizeRotation();
  /**
   * cast SE3Quat into an Isometry3
   */
  explicit operator Isometry3() const;
};

std::ostream& operator<<(std::ostream& out_str, const SE3Quat& se3);

/**
 * @brief TypeTraits specialization for a SE3Quat
 *
 * @tparam
 */
template <>
struct TypeTraits<SE3Quat> {
  enum {  // NOLINT
    kVectorDimension = 7,
    kMinimalVectorDimension = 6,
    kIsVector = 0,
    kIsScalar = 0,
  };
  using Type = SE3Quat;
  using VectorType = VectorN<kVectorDimension>;
  using MinimalVectorType = VectorN<kMinimalVectorDimension>;

  static VectorType toVector(const Type& t) { return t.toVector(); }
  static void toData(const Type& t, double* data) {
    typename VectorType::MapType v(data, kVectorDimension);
    v = t.toVector();
  }

  static VectorType toMinimalVector(const Type& t) { return t.toVector(); }
  static void toMinimalData(const Type& t, double* data) {
    typename MinimalVectorType::MapType v(data, kMinimalVectorDimension);
    v = t.toMinimalVector();
  }

  template <typename Derived>
  static Type fromVector(const Eigen::DenseBase<Derived>& v) {
    Type res;
    res.fromVector(v);
    return res;
  }

  template <typename Derived>
  static Type fromMinimalVector(const Eigen::DenseBase<Derived>& v) {
    Type res;
    res.fromMinimalVector(v);
    return res;
  }
};

}  // namespace g2o

#endif
