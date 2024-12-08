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

#include "se3quat.h"

#include <utility>

#include "g2o/core/eigen_types.h"
#include "se3_ops.h"

namespace g2o {

SE3Quat::SE3Quat() {
  r_.setIdentity();
  t_.setZero();
}

SE3Quat::SE3Quat(const Matrix3& R, Vector3 t)
    : r_(Quaternion(R)), t_(std::move(std::move(t))) {
  normalizeRotation();
}

SE3Quat::SE3Quat(const Quaternion& q, const Vector3& t)  // NOLINT
    : r_(q), t_(t) {
  normalizeRotation();
}

void SE3Quat::setTranslation(const Vector3& t) { t_ = t; }

void SE3Quat::setRotation(const Quaternion& r) { r_ = r; }

SE3Quat SE3Quat::operator*(const SE3Quat& tr2) const {
  SE3Quat result(*this);
  result.t_ += r_ * tr2.t_;
  result.r_ *= tr2.r_;
  result.normalizeRotation();
  return result;
}

SE3Quat& SE3Quat::operator*=(const SE3Quat& tr2) {
  t_ += r_ * tr2.t_;
  r_ *= tr2.r_;
  normalizeRotation();
  return *this;
}

Vector3 SE3Quat::operator*(const Vector3& v) const { return t_ + r_ * v; }

SE3Quat SE3Quat::inverse() const {
  SE3Quat ret;
  ret.r_ = r_.conjugate();
  ret.t_ = ret.r_ * (t_ * -cst(1.));
  return ret;
}

Vector7 SE3Quat::toVector() const {
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

void SE3Quat::fromVector(const Vector7& v) {
  r_ = Quaternion(v[6], v[3], v[4], v[5]);
  t_ = Vector3(v[0], v[1], v[2]);
}

Vector6 SE3Quat::toMinimalVector() const {
  Vector6 v;
  v[0] = t_(0);
  v[1] = t_(1);
  v[2] = t_(2);
  v[3] = r_.x();
  v[4] = r_.y();
  v[5] = r_.z();
  return v;
}

void SE3Quat::fromMinimalVector(const Vector6& v) {
  double w = cst(1.) - v[3] * v[3] - v[4] * v[4] - v[5] * v[5];
  if (w > 0) {
    r_ = Quaternion(std::sqrt(w), v[3], v[4], v[5]);
  } else {
    r_ = Quaternion(0, -v[3], -v[4], -v[5]);
  }
  t_ = Vector3(v[0], v[1], v[2]);
}

Vector6 SE3Quat::log() const {
  Vector6 res;
  const Matrix3 R = r_.toRotationMatrix();
  const double d = cst(0.5) * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
  const Vector3 dR = deltaR(R);

  Vector3 omega;
  Matrix3 V_inv;
  if (std::abs(d) > cst(0.99999)) {
    omega = 0.5 * dR;
    const Matrix3 Omega = skew(omega);
    V_inv = Matrix3::Identity() - cst(0.5) * Omega +
            (cst(1.) / cst(12.)) * (Omega * Omega);
  } else {
    double theta = std::acos(d);
    omega = theta / (2 * std::sqrt(1 - d * d)) * dR;
    const Matrix3 Omega = skew(omega);
    V_inv = (Matrix3::Identity() - cst(0.5) * Omega +
             (1 - theta / (2 * std::tan(theta / 2))) / (theta * theta) *
                 (Omega * Omega));
  }

  const Vector3 upsilon = V_inv * t_;
  for (int i = 0; i < 3; i++) {
    res[i] = omega[i];
  }
  for (int i = 0; i < 3; i++) {
    res[i + 3] = upsilon[i];
  }

  return res;
}

Vector3 SE3Quat::map(const Vector3& xyz) const { return r_ * xyz + t_; }

SE3Quat SE3Quat::exp(const Vector6& update) {
  const Vector3 omega = update.head<3>();
  const Vector3 upsilon = update.tail<3>();
  const double theta = omega.norm();
  const Matrix3 Omega = skew(omega);

  Matrix3 R;
  Matrix3 V;
  const Matrix3 Omega2 = Omega * Omega;
  if (theta < cst(0.00001)) {
    R = (Matrix3::Identity() + Omega + cst(0.5) * Omega2);
    V = (Matrix3::Identity() + cst(0.5) * Omega + cst(1.) / cst(6.) * Omega2);
  } else {
    R = (Matrix3::Identity() + std::sin(theta) / theta * Omega +
         (1 - std::cos(theta)) / (theta * theta) * Omega2);
    V = (Matrix3::Identity() + (1 - std::cos(theta)) / (theta * theta) * Omega +
         (theta - std::sin(theta)) / (std::pow(theta, 3))*Omega2);
  }
  return SE3Quat(Quaternion(R), V * upsilon);
}

Matrix6 SE3Quat::adj() const {
  const Matrix3 R = r_.toRotationMatrix();
  Eigen::Matrix<double, 6, 6, Eigen::ColMajor> res;
  res.block<3, 3>(0, 0) = R;
  res.block<3, 3>(3, 3) = R;
  res.block<3, 3>(3, 0) = skew(t_) * R;
  res.block<3, 3>(0, 3) = Matrix3::Zero(3, 3);
  return res;
}

MatrixN<4> SE3Quat::to_homogeneous_matrix() const {
  MatrixN<4> homogeneous_matrix = MatrixN<4>::Identity();
  homogeneous_matrix.block<3, 3>(0, 0) = r_.toRotationMatrix();
  homogeneous_matrix.col(3).head<3>() = translation();

  return homogeneous_matrix;
}

void SE3Quat::normalizeRotation() {
  if (r_.w() < 0) {
    r_.coeffs() *= -1;
  }
  r_.normalize();
}

SE3Quat::operator Isometry3() const {
  Isometry3 result(rotation());
  result.translation() = translation();
  return result;
}

std::ostream& operator<<(std::ostream& out_str, const SE3Quat& se3) {
  out_str << se3.to_homogeneous_matrix();
  return out_str;
}

}  // namespace g2o
