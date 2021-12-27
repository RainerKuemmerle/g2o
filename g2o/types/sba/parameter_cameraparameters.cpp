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

#include "g2o/types/slam3d/se3_ops.h"

namespace g2o {

CameraParameters::CameraParameters()
    : focal_length(1.), principle_point(Vector2(0., 0.)), baseline(0.5) {}

CameraParameters::CameraParameters(number_t focal_length,
                                   const Vector2 &principle_point,
                                   number_t baseline)
    : focal_length(focal_length),
      principle_point(principle_point),
      baseline(baseline) {}

bool CameraParameters::read(std::istream &is) {
  is >> focal_length;
  is >> principle_point[0];
  is >> principle_point[1];
  is >> baseline;
  return true;
}

bool CameraParameters::write(std::ostream &os) const {
  os << focal_length << " ";
  os << principle_point.x() << " ";
  os << principle_point.y() << " ";
  os << baseline << " ";
  return true;
}

Vector2 CameraParameters::cam_map(const Vector3 &trans_xyz) const {
  Vector2 proj = project(trans_xyz);
  Vector2 res;
  res[0] = proj[0] * focal_length + principle_point[0];
  res[1] = proj[1] * focal_length + principle_point[1];
  return res;
}

Vector3 CameraParameters::stereocam_uvu_map(const Vector3 &trans_xyz) const {
  Vector2 uv_left = cam_map(trans_xyz);
  number_t proj_x_right = (trans_xyz[0] - baseline) / trans_xyz[2];
  number_t u_right = proj_x_right * focal_length + principle_point[0];
  return Vector3(uv_left[0], uv_left[1], u_right);
}

}  // namespace g2o
