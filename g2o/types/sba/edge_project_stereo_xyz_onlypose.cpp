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

#include "edge_project_stereo_xyz_onlypose.h"

#include <Eigen/Core>

#include "g2o/types/sba/vertex_se3_expmap.h"

namespace g2o {

void EdgeStereoSE3ProjectXYZOnlyPose::linearizeOplus() {
  VertexSE3Expmap* vi = vertexXnRaw<0>();
  Vector3 xyz_trans = vi->estimate().map(Xw);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double invz = 1.0 / xyz_trans[2];
  double invz_2 = invz * invz;

  jacobianOplusXi_(0, 0) = x * y * invz_2 * fx;
  jacobianOplusXi_(0, 1) = -(1 + (x * x * invz_2)) * fx;
  jacobianOplusXi_(0, 2) = y * invz * fx;
  jacobianOplusXi_(0, 3) = -invz * fx;
  jacobianOplusXi_(0, 4) = 0;
  jacobianOplusXi_(0, 5) = x * invz_2 * fx;

  jacobianOplusXi_(1, 0) = (1 + y * y * invz_2) * fy;
  jacobianOplusXi_(1, 1) = -x * y * invz_2 * fy;
  jacobianOplusXi_(1, 2) = -x * invz * fy;
  jacobianOplusXi_(1, 3) = 0;
  jacobianOplusXi_(1, 4) = -invz * fy;
  jacobianOplusXi_(1, 5) = y * invz_2 * fy;

  jacobianOplusXi_(2, 0) = jacobianOplusXi_(0, 0) - bf * y * invz_2;
  jacobianOplusXi_(2, 1) = jacobianOplusXi_(0, 1) + bf * x * invz_2;
  jacobianOplusXi_(2, 2) = jacobianOplusXi_(0, 2);
  jacobianOplusXi_(2, 3) = jacobianOplusXi_(0, 3);
  jacobianOplusXi_(2, 4) = 0;
  jacobianOplusXi_(2, 5) = jacobianOplusXi_(0, 5) - bf * invz_2;
}

void EdgeStereoSE3ProjectXYZOnlyPose::computeError() {
  const VertexSE3Expmap* v1 = vertexXnRaw<0>();
  Vector3 obs(measurement_);
  error_ = obs - cam_project(v1->estimate().map(Xw));
}

bool EdgeStereoSE3ProjectXYZOnlyPose::isDepthPositive() {
  const VertexSE3Expmap* v1 = vertexXnRaw<0>();
  return (v1->estimate().map(Xw))(2) > 0;
}

Vector3 EdgeStereoSE3ProjectXYZOnlyPose::cam_project(
    const Vector3& trans_xyz) const {
  const float invz = 1.0F / trans_xyz[2];
  Vector3 res;
  res[0] = trans_xyz[0] * invz * fx + cx;
  res[1] = trans_xyz[1] * invz * fy + cy;
  res[2] = res[0] - bf * invz;
  return res;
}

}  // namespace g2o
