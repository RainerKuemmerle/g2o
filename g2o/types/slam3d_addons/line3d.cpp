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

#include "line3d.h"

#include <Eigen/Cholesky>

#include "g2o/core/eigen_types.h"
#include "g2o/stuff/misc.h"

namespace g2o {

namespace {
inline Matrix3 skew(const Vector3& t) {
  Matrix3 S;
  S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
  return S;
}
}  // namespace

Line3D::Line3D() { line << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0; }

Line3D::Line3D(const Vector6& v) : line(v) {}  // NOLINT

void Line3D::setW(const Vector3& w_) { line.head<3>() = w_; }

void Line3D::setD(const Vector3& d_) { line.tail<3>() = d_; }

Line3D Line3D::fromCartesian(const Vector6& cart) {
  Line3D l;
  Vector3 _p = cart.head<3>();
  Vector3 _d = cart.tail<3>() * 1.0 / cart.tail<3>().norm();
  _p -= _d * (_d.dot(_p));
  l.setW(_p.cross(_p + _d));
  l.setD(_d);
  return l;
}

Vector6 Line3D::toCartesian() const {
  Vector6 cartesian;
  cartesian.tail<3>() = d() / d().norm();
  Matrix3 W = -skew(d());
  double damping = cst(1e-9);
  Matrix3 A = W.transpose() * W + (Matrix3::Identity() * damping);
  cartesian.head<3>() = A.ldlt().solve(W.transpose() * w());
  return cartesian;
}

Line3D Line3D::fromOrthonormal(const OrthonormalLine3D& ortho) {
  Vector3 w;
  w.x() = ortho.U(0, 0) * ortho.W(0, 0);
  w.y() = ortho.U(1, 0) * ortho.W(0, 0);
  w.z() = ortho.U(2, 0) * ortho.W(0, 0);

  Vector3 d;
  d.x() = ortho.U(0, 1) * ortho.W(1, 0);
  d.y() = ortho.U(1, 1) * ortho.W(1, 0);
  d.z() = ortho.U(2, 1) * ortho.W(1, 0);

  Line3D l;
  l.setW(w);
  l.setD(d);
  l.normalize();

  return l;
}

OrthonormalLine3D Line3D::toOrthonormal() const {
  OrthonormalLine3D ortho;

  Vector2 mags;
  mags << d().norm(), w().norm();

  double wn = 1.0 / mags.norm();
  ortho.W << mags.y() * wn, -mags.x() * wn, mags.x() * wn, mags.y() * wn;

  double mn = 1.0 / mags.y();
  double dn = 1.0 / mags.x();
  Vector3 mdcross;
  mdcross = w().cross(d());
  double mdcrossn = 1.0 / mdcross.norm();
  ortho.U << w().x() * mn, d().x() * dn, mdcross.x() * mdcrossn, w().y() * mn,
      d().y() * dn, mdcross.y() * mdcrossn, w().z() * mn, d().z() * dn,
      mdcross.z() * mdcrossn;

  return ortho;
}

void Line3D::normalize() {
  double n = 1.0 / d().norm();
  line *= n;
}

Line3D Line3D::normalized() const {
  const Vector6 result = line * (1.0 / d().norm());
  return Line3D(result);
}

void Line3D::oplus(const Vector4& v) {
  OrthonormalLine3D ortho_estimate = toOrthonormal();
  OrthonormalLine3D ortho_update;
  ortho_update.W << std::cos(v[3]), -std::sin(v[3]), std::sin(v[3]),
      std::cos(v[3]);
  Quaternion quat(std::sqrt(1 - v.head<3>().squaredNorm()), v[0], v[1], v[2]);
  quat.normalize();
  ortho_update.U = quat.toRotationMatrix();

  ortho_estimate.U = ortho_estimate.U * ortho_update.U;
  ortho_estimate.W = ortho_estimate.W * ortho_update.W;

  *this = fromOrthonormal(ortho_estimate);
  this->normalize();
}

[[nodiscard]] Vector4 Line3D::ominus(const Line3D& line) const {
  OrthonormalLine3D ortho_estimate = toOrthonormal();
  OrthonormalLine3D ortho_line = line.toOrthonormal();

  Matrix2 W_delta = ortho_estimate.W.transpose() * ortho_line.W;
  Matrix3 U_delta = ortho_estimate.U.transpose() * ortho_line.U;

  Vector4 delta;
  Quaternion q(U_delta);
  q.normalize();
  delta[0] = q.x();
  delta[1] = q.y();
  delta[2] = q.z();
  delta[3] = std::atan2(W_delta(1, 0), W_delta(0, 0));

  return delta;
}

Line3D operator*(const Isometry3& t, const Line3D& line) {
  Matrix6 A = Matrix6::Zero();
  A.block<3, 3>(0, 0) = t.linear();
  A.block<3, 3>(0, 3) = skew(t.translation()) * t.linear();
  A.block<3, 3>(3, 3) = t.linear();
  return Line3D(A * line.line);
}

namespace internal {
Vector6 transformCartesianLine(const Isometry3& t, const Vector6& line) {
  Vector6 l;
  l.head<3>() = t * line.head<3>();
  l.tail<3>() = t.linear() * line.tail<3>();
  return normalizeCartesianLine(l);
}

Vector6 normalizeCartesianLine(const Vector6& line) {
  Vector3 p0 = line.head<3>();
  Vector3 d0 = line.tail<3>();
  d0.normalize();
  p0 -= d0 * (d0.dot(p0));
  Vector6 nl;
  nl.head<3>() = p0;
  nl.tail<3>() = d0;
  return nl;
}

double mline_elevation(const double v[3]) {
  return std::atan2(v[2], sqrt(v[0] * v[0] + v[1] * v[1]));
}

double getAzimuth(const Vector3& direction) {
  return std::atan2(direction.y(), direction.x());
}

double getElevation(const Vector3& direction) {
  return std::atan2(direction.z(), direction.head<2>().norm());
}

}  // namespace internal

}  // namespace g2o
