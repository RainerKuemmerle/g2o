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

#include "g2o/types/sba/edge_se3_expmap.h"

#include "g2o/types/sba/vertex_se3_expmap.h"
#include "g2o/types/slam3d/se3_ops.h"

namespace g2o {
namespace {

Matrix3 so3LeftJacobianInverse(const Vector3& omega) {
  const double theta_sq = omega.squaredNorm();
  const Matrix3 Omega = skew(omega);
  const Matrix3 Omega_sq = Omega * Omega;

  if (theta_sq < cst(1e-10)) {
    return Matrix3::Identity() - cst(0.5) * Omega + cst(1. / 12.) * Omega_sq;
  }

  const double theta = std::sqrt(theta_sq);
  const double half_theta = cst(0.5) * theta;
  return Matrix3::Identity() - cst(0.5) * Omega +
         ((cst(1.) - (half_theta / std::tan(half_theta))) / theta_sq) *
             Omega_sq;
}

Matrix3 se3JacobianUpperRightBlock(const Vector3& upsilon,
                                   const Vector3& omega) {
  const double theta_sq = omega.squaredNorm();
  const Matrix3 Upsilon = skew(upsilon);

  if (theta_sq < cst(1e-10)) {
    return cst(0.5) * Upsilon;
  }

  const double theta = std::sqrt(theta_sq);
  const double inv_theta = cst(1.) / theta;
  const double inv_theta_sq = inv_theta * inv_theta;
  const double inv_theta_4 = inv_theta_sq * inv_theta_sq;
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double c1 = inv_theta_sq - (sin_theta * inv_theta_sq * inv_theta);
  const double c2 =
      (cst(0.5) * inv_theta_sq) + (cos_theta * inv_theta_4) - inv_theta_4;
  const double c3 = inv_theta_4 + (cst(0.5) * cos_theta * inv_theta_4) -
                    (cst(1.5) * sin_theta * inv_theta * inv_theta_4);

  const Matrix3 Omega = skew(omega);
  const Matrix3 OmegaUpsilon = Omega * Upsilon;
  const Matrix3 OmegaUpsilonOmega = OmegaUpsilon * Omega;

  return cst(0.5) * Upsilon +
         c1 * (OmegaUpsilon + Upsilon * Omega + OmegaUpsilonOmega) -
         c2 * (theta_sq * Upsilon + cst(2.) * OmegaUpsilonOmega) +
         c3 * (OmegaUpsilonOmega * Omega + Omega * OmegaUpsilonOmega);
}

Matrix6 se3LeftJacobianInverse(const Vector6& xi) {
  const Vector3 omega = xi.head<3>();
  const Vector3 upsilon = xi.tail<3>();

  const Matrix3 J_inv = so3LeftJacobianInverse(omega);
  const Matrix3 Q = se3JacobianUpperRightBlock(upsilon, omega);

  Matrix6 result = Matrix6::Zero();
  result.block<3, 3>(0, 0) = J_inv;
  result.block<3, 3>(3, 0) = -J_inv * Q * J_inv;
  result.block<3, 3>(3, 3) = J_inv;
  return result;
}

Matrix6 se3CurrentLogDerivative(const SE3Quat& transform) {
  const Matrix3 rotation = transform.rotation().toRotationMatrix();
  const double trace = rotation.trace();
  const double d = cst(0.5) * (trace - cst(1.));
  if (std::abs(d) <= cst(0.99999)) {
    return se3LeftJacobianInverse(transform.log());
  }

  const Vector3 omega = cst(0.5) * deltaR(rotation);
  const Matrix3 Omega = skew(omega);
  const Matrix3 M =
      Matrix3::Identity() - cst(0.5) * Omega + cst(1. / 12.) * (Omega * Omega);
  const Matrix3 rotational_block =
      cst(0.5) * (trace * Matrix3::Identity() - rotation.transpose());

  Matrix3 omega_to_upsilon;
  const Vector3& translation = transform.translation();
  for (int axis = 0; axis < 3; ++axis) {
    Vector3 basis = Vector3::Zero();
    basis[axis] = cst(1.);
    const Matrix3 basis_hat = skew(basis);
    omega_to_upsilon.col(axis) =
        (-cst(0.5) * basis_hat +
         cst(1. / 12.) * (basis_hat * Omega + Omega * basis_hat)) *
        translation;
  }

  Matrix6 result = Matrix6::Zero();
  result.block<3, 3>(0, 0) = rotational_block;
  result.block<3, 3>(3, 0) =
      M * (-skew(translation)) + omega_to_upsilon * rotational_block;
  result.block<3, 3>(3, 3) = M;
  return result;
}

}  // namespace

void EdgeSE3Expmap::computeError() {
  const VertexSE3Expmap* v1 = vertexXnRaw<0>();
  const VertexSE3Expmap* v2 = vertexXnRaw<1>();

  SE3Quat C(measurement_);
  SE3Quat err = v2->estimate().inverse() * C * v1->estimate();
  error_ = err.log();
}

void EdgeSE3Expmap::linearizeOplus() {
  const VertexSE3Expmap* v1 = vertexXnRaw<0>();
  const VertexSE3Expmap* v2 = vertexXnRaw<1>();

  const SE3Quat Ti(v1->estimate());
  const SE3Quat Tj(v2->estimate());
  const SE3Quat A = Tj.inverse() * measurement_;
  const SE3Quat E = A * Ti;
  const Matrix6 log_derivative = se3CurrentLogDerivative(E);

  jacobianOplusXi_ = log_derivative * A.adj();
  jacobianOplusXj_ = -log_derivative * Tj.inverse().adj();
}

}  // namespace g2o
