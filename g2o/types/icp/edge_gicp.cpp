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

#include "edge_gicp.h"

#include <Eigen/Geometry>

#include "g2o/core/eigen_types.h"

namespace g2o {

namespace {
constexpr double kYSquaredBnd{0.1};

void makeRot(Matrix3& R, const Vector3& normal) {
  Vector3 y(0., 1., 0.);
  R.row(2) = normal;
  y = y - normal(1) * normal;
  const double ysquarednorm = y.squaredNorm();
  if (ysquarednorm >= kYSquaredBnd) {
    y /= std::sqrt(ysquarednorm);
    R.row(1) = y;
    R.row(0) = normal.cross(R.row(1));
  } else {
    Vector3 x(-1., 0., 0.);
    x = x + normal(0) * normal;
    x.normalize();
    R.row(0) = x;
    R.row(1) = -normal.cross(R.row(0));
  }
}

Matrix3 prec(double e, const Matrix3& R) {
  Matrix3 prec_mat;
  prec_mat << e, 0, 0, 0, e, 0, 0, 0, 1;
  return R.transpose() * prec_mat * R;
}

Matrix3 cov(double e, const Matrix3& R) {
  Matrix3 cov_mat;
  cov_mat << 1, 0, 0, 0, 1, 0, 0, 0, e;
  return R.transpose() * cov_mat * R;
}
}  // namespace

EdgeGICP::EdgeGICP() {
  pos0.setZero();
  pos1.setZero();
  normal0 << 0, 0, 1;
  normal1 << 0, 0, 1;
  // makeRot();
  R0.setIdentity();
  R1.setIdentity();
}

void EdgeGICP::makeRot0() { makeRot(R0, normal0); }

void EdgeGICP::makeRot1() { makeRot(R1, normal1); }

Matrix3 EdgeGICP::prec0(double e) {
  makeRot0();
  return prec(e, R0);
}

Matrix3 EdgeGICP::prec1(double e) {
  makeRot1();
  return prec(e, R1);
}

Matrix3 EdgeGICP::cov0(double e) {
  makeRot0();
  return cov(e, R0);
}

Matrix3 EdgeGICP::cov1(double e) {
  makeRot1();
  return cov(e, R1);
}

}  // namespace g2o
