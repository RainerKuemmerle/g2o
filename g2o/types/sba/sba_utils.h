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

#ifndef G2O_SBA_UTILS_H
#define G2O_SBA_UTILS_H

#include "g2o/types/slam3d/se3_ops.h"
#include "g2o_types_sba_api.h"

namespace g2o {

namespace internal {

inline Vector3 invert_depth(const Vector3& x) {
  Vector2 aux = x.head<2>();
  return unproject(aux) / x[2];
}

inline Eigen::Matrix<number_t, 2, 3, Eigen::ColMajor> d_proj_d_y(
    const number_t& f, const Vector3& xyz) {
  number_t z_sq = xyz[2] * xyz[2];
  Eigen::Matrix<number_t, 2, 3, Eigen::ColMajor> J;
  J << f / xyz[2], 0, -(f * xyz[0]) / z_sq, 0, f / xyz[2], -(f * xyz[1]) / z_sq;
  return J;
}

inline Eigen::Matrix<number_t, 3, 6, Eigen::ColMajor> d_expy_d_y(
    const Vector3& y) {
  Eigen::Matrix<number_t, 3, 6, Eigen::ColMajor> J;
  J.topLeftCorner<3, 3>() = -skew(y);
  J.bottomRightCorner<3, 3>().setIdentity();

  return J;
}

inline Matrix3 d_Tinvpsi_d_psi(const SE3Quat& T, const Vector3& psi) {
  Matrix3 R = T.rotation().toRotationMatrix();
  Vector3 x = invert_depth(psi);
  Vector3 r1 = R.col(0);
  Vector3 r2 = R.col(1);
  Matrix3 J;
  J.col(0) = r1;
  J.col(1) = r2;
  J.col(2) = -R * x;
  J *= 1. / psi.z();
  return J;
}

}  // namespace internal
}  // namespace g2o

#endif
