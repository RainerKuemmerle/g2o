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
#include <utility>

#include "g2o/stuff/misc.h"
#include "se3_ops.h"

namespace g2o {

class G2O_TYPES_SLAM3D_API SE3Quat {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:
  Quaternion r_;
  Vector3 t_;

 public:
  SE3Quat() {
    r_.setIdentity();
    t_.setZero();
  }

  SE3Quat(const Matrix3& R, Vector3 t)
      : r_(Quaternion(R)), t_(std::move(std::move(t))) {
    normalizeRotation();
  }

  SE3Quat(Quaternion q, Vector3 t) : r_(std::move(q)), t_(std::move(std::move(t))) {
    normalizeRotation();
  }

  /**
   * templaized constructor which allows v to be an arbitrary Eigen Vector type,
   * e.g., Vector6 or Map<Vector6>
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
        number_t w2 = cst(1.) - r_.squaredNorm();
        r_.w() = (w2 < cst(0.)) ? cst(0.) : std::sqrt(w2);
      }
    } else if (v.size() == 7) {
      int idx = 0;
      for (int i = 0; i < 3; ++i, ++idx) t_(i) = v(idx);
      for (int i = 0; i < 4; ++i, ++idx) r_.coeffs()(i) = v(idx);
      normalizeRotation();
    }
  }

  inline const Vector3& translation() const { return t_; }

  inline void setTranslation(const Vector3& t) { t_ = t; }

  inline const Quaternion& rotation() const { return r_; }

  void setRotation(const Quaternion& r) { r_ = r; }

  inline SE3Quat operator*(const SE3Quat& tr2) const {
    SE3Quat result(*this);
    result.t_ += r_ * tr2.t_;
    result.r_ *= tr2.r_;
    result.normalizeRotation();
    return result;
  }

  inline SE3Quat& operator*=(const SE3Quat& tr2) {
    t_ += r_ * tr2.t_;
    r_ *= tr2.r_;
    normalizeRotation();
    return *this;
  }

  inline Vector3 operator*(const Vector3& v) const { return t_ + r_ * v; }

  inline SE3Quat inverse() const {
    SE3Quat ret;
    ret.r_ = r_.conjugate();
    ret.t_ = ret.r_ * (t_ * -cst(1.));
    return ret;
  }

  inline number_t operator[](int i) const {
    assert(i < 7);
    if (i < 3) return t_[i];
    return r_.coeffs()[i - 3];
  }

  inline Vector7 toVector() const {
    Vector7 v;
    v[0] = t_(0);
    v[1] = t_(1);
    v[2] = t_(2);
    v[3] = r_.x();
    v[4] = r_.y();
    v[5] = r_.z();
    v[6] = r_.w();
    return v;
  }

  inline void fromVector(const Vector7& v) {
    r_ = Quaternion(v[6], v[3], v[4], v[5]);
    t_ = Vector3(v[0], v[1], v[2]);
  }

  inline Vector6 toMinimalVector() const {
    Vector6 v;
    v[0] = t_(0);
    v[1] = t_(1);
    v[2] = t_(2);
    v[3] = r_.x();
    v[4] = r_.y();
    v[5] = r_.z();
    return v;
  }

  inline void fromMinimalVector(const Vector6& v) {
    number_t w = cst(1.) - v[3] * v[3] - v[4] * v[4] - v[5] * v[5];
    if (w > 0) {
      r_ = Quaternion(std::sqrt(w), v[3], v[4], v[5]);
    } else {
      r_ = Quaternion(0, -v[3], -v[4], -v[5]);
    }
    t_ = Vector3(v[0], v[1], v[2]);
  }

  Vector6 log() const {
    Vector6 res;
    Matrix3 R = r_.toRotationMatrix();
    number_t d = cst(0.5) * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
    Vector3 omega;
    Vector3 upsilon;

    Vector3 dR = deltaR(R);
    Matrix3 V_inv;

    if (std::abs(d) > cst(0.99999)) {
      omega = 0.5 * dR;
      Matrix3 Omega = skew(omega);
      V_inv = Matrix3::Identity() - cst(0.5) * Omega +
              (cst(1.) / cst(12.)) * (Omega * Omega);
    } else {
      number_t theta = std::acos(d);
      omega = theta / (2 * std::sqrt(1 - d * d)) * dR;
      Matrix3 Omega = skew(omega);
      V_inv = (Matrix3::Identity() - cst(0.5) * Omega +
               (1 - theta / (2 * std::tan(theta / 2))) / (theta * theta) *
                   (Omega * Omega));
    }

    upsilon = V_inv * t_;
    for (int i = 0; i < 3; i++) {
      res[i] = omega[i];
    }
    for (int i = 0; i < 3; i++) {
      res[i + 3] = upsilon[i];
    }

    return res;
  }

  Vector3 map(const Vector3& xyz) const { return r_ * xyz + t_; }

  static SE3Quat exp(const Vector6& update) {
    Vector3 omega;
    for (int i = 0; i < 3; i++) omega[i] = update[i];
    Vector3 upsilon;
    for (int i = 0; i < 3; i++) upsilon[i] = update[i + 3];

    number_t theta = omega.norm();
    Matrix3 Omega = skew(omega);

    Matrix3 R;
    Matrix3 V;
    if (theta < cst(0.00001)) {
      Matrix3 Omega2 = Omega * Omega;

      R = (Matrix3::Identity() + Omega + cst(0.5) * Omega2);

      V = (Matrix3::Identity() + cst(0.5) * Omega + cst(1.) / cst(6.) * Omega2);
    } else {
      Matrix3 Omega2 = Omega * Omega;

      R = (Matrix3::Identity() + std::sin(theta) / theta * Omega +
           (1 - std::cos(theta)) / (theta * theta) * Omega2);

      V = (Matrix3::Identity() +
           (1 - std::cos(theta)) / (theta * theta) * Omega +
           (theta - std::sin(theta)) / (std::pow(theta, 3)) * Omega2);
    }
    return SE3Quat(Quaternion(R), V * upsilon);
  }

  Eigen::Matrix<number_t, 6, 6, Eigen::ColMajor> adj() const {
    Matrix3 R = r_.toRotationMatrix();
    Eigen::Matrix<number_t, 6, 6, Eigen::ColMajor> res;
    res.block(0, 0, 3, 3) = R;
    res.block(3, 3, 3, 3) = R;
    res.block(3, 0, 3, 3) = skew(t_) * R;
    res.block(0, 3, 3, 3) = Matrix3::Zero(3, 3);
    return res;
  }

  Eigen::Matrix<number_t, 4, 4, Eigen::ColMajor> to_homogeneous_matrix() const {
    Eigen::Matrix<number_t, 4, 4, Eigen::ColMajor> homogeneous_matrix;
    homogeneous_matrix.setIdentity();
    homogeneous_matrix.block(0, 0, 3, 3) = r_.toRotationMatrix();
    homogeneous_matrix.col(3).head(3) = translation();

    return homogeneous_matrix;
  }

  void normalizeRotation() {
    if (r_.w() < 0) {
      r_.coeffs() *= -1;
    }
    r_.normalize();
  }

  /**
   * cast SE3Quat into an Isometry3
   */
  explicit operator Isometry3() const {
    Isometry3 result(rotation());
    result.translation() = translation();
    return result;
  }
};

inline std::ostream& operator<<(std::ostream& out_str, const SE3Quat& se3) {
  out_str << se3.to_homogeneous_matrix() << std::endl;
  return out_str;
}

// G2O_TYPES_SLAM3D_API Quaternion euler_to_quat(number_t yaw, number_t pitch,
// number_t roll); G2O_TYPES_SLAM3D_API void quat_to_euler(const Quaternion& q,
// number_t& yaw, number_t& pitch, number_t& roll); G2O_TYPES_SLAM3D_API void
// jac_quat3_euler3(Eigen::Matrix<number_t, 6, 6, Eigen::ColMajor>& J, const
// SE3Quat& t);

}  // namespace g2o

#endif
