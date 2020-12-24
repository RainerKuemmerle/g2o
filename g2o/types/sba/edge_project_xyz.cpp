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

#include "edge_project_xyz.h"

namespace g2o {

EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ()
    : BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexSE3Expmap>() {}

bool EdgeSE3ProjectXYZ::read(std::istream &is) {
  internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeSE3ProjectXYZ::write(std::ostream &os) const {
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

void EdgeSE3ProjectXYZ::computeError() {
  const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
  const VertexPointXYZ *v2 = static_cast<const VertexPointXYZ *>(_vertices[0]);
  Vector2 obs(_measurement);
  _error = obs - cam_project(v1->estimate().map(v2->estimate()));
}

bool EdgeSE3ProjectXYZ::isDepthPositive() {
  const VertexSE3Expmap *v1 = static_cast<const VertexSE3Expmap *>(_vertices[1]);
  const VertexPointXYZ *v2 = static_cast<const VertexPointXYZ *>(_vertices[0]);
  return (v1->estimate().map(v2->estimate()))(2) > 0.0;
}

void EdgeSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap *vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexPointXYZ *vi = static_cast<VertexPointXYZ *>(_vertices[0]);
  Vector3 xyz = vi->estimate();
  Vector3 xyz_trans = T.map(xyz);

  number_t x = xyz_trans[0];
  number_t y = xyz_trans[1];
  number_t z = xyz_trans[2];
  number_t z_2 = z * z;

  Eigen::Matrix<number_t, 2, 3> tmp;
  tmp(0, 0) = fx;
  tmp(0, 1) = 0;
  tmp(0, 2) = -x / z * fx;

  tmp(1, 0) = 0;
  tmp(1, 1) = fy;
  tmp(1, 2) = -y / z * fy;

  _jacobianOplusXi = -1. / z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0, 0) = x * y / z_2 * fx;
  _jacobianOplusXj(0, 1) = -(1 + (x * x / z_2)) * fx;
  _jacobianOplusXj(0, 2) = y / z * fx;
  _jacobianOplusXj(0, 3) = -1. / z * fx;
  _jacobianOplusXj(0, 4) = 0;
  _jacobianOplusXj(0, 5) = x / z_2 * fx;

  _jacobianOplusXj(1, 0) = (1 + y * y / z_2) * fy;
  _jacobianOplusXj(1, 1) = -x * y / z_2 * fy;
  _jacobianOplusXj(1, 2) = -x / z * fy;
  _jacobianOplusXj(1, 3) = 0;
  _jacobianOplusXj(1, 4) = -1. / z * fy;
  _jacobianOplusXj(1, 5) = y / z_2 * fy;
}

Vector2 EdgeSE3ProjectXYZ::cam_project(const Vector3 &trans_xyz) const {
  Vector2 proj = project(trans_xyz);
  Vector2 res;
  res[0] = proj[0] * fx + cx;
  res[1] = proj[1] * fy + cy;
  return res;
}

}  // namespace g2o
