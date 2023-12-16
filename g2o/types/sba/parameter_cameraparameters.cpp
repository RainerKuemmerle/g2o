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

#include "parameter_cameraparameters.h"

#include <Eigen/Core>
#include <iostream>

#include "g2o/types/slam3d/se3_ops.h"

namespace g2o {

CameraParameters::CameraParameters(
    double focalLength, const Eigen::Ref<const Vector2>& principlePoint,
    double baseLine) {
  parameter_.focal_length = focalLength;
  parameter_.principle_point = principlePoint;
  parameter_.baseline = baseLine;
}

bool CameraParameters::read(std::istream& is) {
  is >> parameter_.focal_length;
  is >> parameter_.principle_point[0];
  is >> parameter_.principle_point[1];
  is >> parameter_.baseline;
  return true;
}

bool CameraParameters::write(std::ostream& os) const {
  os << parameter_.focal_length << " ";
  os << parameter_.principle_point.x() << " ";
  os << parameter_.principle_point.y() << " ";
  os << parameter_.baseline << " ";
  return true;
}

Vector2 CameraParameters::cam_map(const Vector3& trans_xyz) const {
  Vector2 proj = project(trans_xyz);
  Vector2 res;
  res[0] = proj[0] * parameter_.focal_length + parameter_.principle_point[0];
  res[1] = proj[1] * parameter_.focal_length + parameter_.principle_point[1];
  return res;
}

Vector3 CameraParameters::stereocam_uvu_map(const Vector3& trans_xyz) const {
  Vector2 uv_left = cam_map(trans_xyz);
  double proj_x_right = (trans_xyz[0] - parameter_.baseline) / trans_xyz[2];
  double u_right =
      proj_x_right * parameter_.focal_length + parameter_.principle_point[0];
  return Vector3(uv_left[0], uv_left[1], u_right);
}

}  // namespace g2o
