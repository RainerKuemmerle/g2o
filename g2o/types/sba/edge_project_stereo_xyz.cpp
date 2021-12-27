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

#include "edge_project_stereo_xyz.h"

namespace g2o {

Vector3 EdgeStereoSE3ProjectXYZ::cam_project(const Vector3 &trans_xyz,
                                             const float &bf) const {
  const number_t invz = 1.0F / trans_xyz[2];
  Vector3 res;
  res[0] = trans_xyz[0] * invz * fx + cx;
  res[1] = trans_xyz[1] * invz * fy + cy;
  res[2] = res[0] - bf * invz;
  return res;
}

bool EdgeStereoSE3ProjectXYZ::read(std::istream &is) {
  internal::readVector(is, measurement_);
  return readInformationMatrix(is);
}

bool EdgeStereoSE3ProjectXYZ::write(std::ostream &os) const {
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeStereoSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap *vj = vertexXnRaw<1>();
  SE3Quat T(vj->estimate());
  VertexPointXYZ *vi = vertexXnRaw<0>();
  Vector3 xyz = vi->estimate();
  Vector3 xyz_trans = T.map(xyz);

  const Matrix3 R = T.rotation().toRotationMatrix();

  number_t x = xyz_trans[0];
  number_t y = xyz_trans[1];
  number_t z = xyz_trans[2];
  number_t z_2 = z * z;

  jacobianOplusXi_(0, 0) = -fx * R(0, 0) / z + fx * x * R(2, 0) / z_2;
  jacobianOplusXi_(0, 1) = -fx * R(0, 1) / z + fx * x * R(2, 1) / z_2;
  jacobianOplusXi_(0, 2) = -fx * R(0, 2) / z + fx * x * R(2, 2) / z_2;

  jacobianOplusXi_(1, 0) = -fy * R(1, 0) / z + fy * y * R(2, 0) / z_2;
  jacobianOplusXi_(1, 1) = -fy * R(1, 1) / z + fy * y * R(2, 1) / z_2;
  jacobianOplusXi_(1, 2) = -fy * R(1, 2) / z + fy * y * R(2, 2) / z_2;

  jacobianOplusXi_(2, 0) = jacobianOplusXi_(0, 0) - bf * R(2, 0) / z_2;
  jacobianOplusXi_(2, 1) = jacobianOplusXi_(0, 1) - bf * R(2, 1) / z_2;
  jacobianOplusXi_(2, 2) = jacobianOplusXi_(0, 2) - bf * R(2, 2) / z_2;

  jacobianOplusXj_(0, 0) = x * y / z_2 * fx;
  jacobianOplusXj_(0, 1) = -(1 + (x * x / z_2)) * fx;
  jacobianOplusXj_(0, 2) = y / z * fx;
  jacobianOplusXj_(0, 3) = -1. / z * fx;
  jacobianOplusXj_(0, 4) = 0;
  jacobianOplusXj_(0, 5) = x / z_2 * fx;

  jacobianOplusXj_(1, 0) = (1 + y * y / z_2) * fy;
  jacobianOplusXj_(1, 1) = -x * y / z_2 * fy;
  jacobianOplusXj_(1, 2) = -x / z * fy;
  jacobianOplusXj_(1, 3) = 0;
  jacobianOplusXj_(1, 4) = -1. / z * fy;
  jacobianOplusXj_(1, 5) = y / z_2 * fy;

  jacobianOplusXj_(2, 0) = jacobianOplusXj_(0, 0) - bf * y / z_2;
  jacobianOplusXj_(2, 1) = jacobianOplusXj_(0, 1) + bf * x / z_2;
  jacobianOplusXj_(2, 2) = jacobianOplusXj_(0, 2);
  jacobianOplusXj_(2, 3) = jacobianOplusXj_(0, 3);
  jacobianOplusXj_(2, 4) = 0;
  jacobianOplusXj_(2, 5) = jacobianOplusXj_(0, 5) - bf / z_2;
}

}  // namespace g2o
