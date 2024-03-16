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

#include "edge_project_xyz2uv.h"

#include <Eigen/Core>

#include "g2o/types/sba/parameter_cameraparameters.h"
#include "g2o/types/sba/vertex_se3_expmap.h"
#include "g2o/types/slam3d/se3quat.h"

namespace g2o {

EdgeProjectXYZ2UV::EdgeProjectXYZ2UV() {
  resizeParameters(1);
  installParameter<CameraParameters>(0);
}

void EdgeProjectXYZ2UV::computeError() {
  const VertexSE3Expmap* v1 = vertexXnRaw<1>();
  const VertexPointXYZ* v2 = vertexXnRaw<0>();
  auto cam = std::static_pointer_cast<CameraParameters>(parameter(0));
  error_ = measurement() - cam->cam_map(v1->estimate().map(v2->estimate()));
}

void EdgeProjectXYZ2UV::linearizeOplus() {
  VertexSE3Expmap* vj = vertexXnRaw<1>();
  SE3Quat T(vj->estimate());
  VertexPointXYZ* vi = vertexXnRaw<0>();
  Vector3 xyz = vi->estimate();
  Vector3 xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z * z;

  const StereoCameraParameters& cam =
      static_cast<CameraParameters*>(parameter(0).get())->param();

  Eigen::Matrix<double, 2, 3, Eigen::ColMajor> tmp;
  tmp(0, 0) = cam.focal_length;
  tmp(0, 1) = 0;
  tmp(0, 2) = -x / z * cam.focal_length;

  tmp(1, 0) = 0;
  tmp(1, 1) = cam.focal_length;
  tmp(1, 2) = -y / z * cam.focal_length;

  jacobianOplusXi_ = -1. / z * tmp * T.rotation().toRotationMatrix();

  jacobianOplusXj_(0, 0) = x * y / z_2 * cam.focal_length;
  jacobianOplusXj_(0, 1) = -(1 + (x * x / z_2)) * cam.focal_length;
  jacobianOplusXj_(0, 2) = y / z * cam.focal_length;
  jacobianOplusXj_(0, 3) = -1. / z * cam.focal_length;
  jacobianOplusXj_(0, 4) = 0;
  jacobianOplusXj_(0, 5) = x / z_2 * cam.focal_length;

  jacobianOplusXj_(1, 0) = (1 + y * y / z_2) * cam.focal_length;
  jacobianOplusXj_(1, 1) = -x * y / z_2 * cam.focal_length;
  jacobianOplusXj_(1, 2) = -x / z * cam.focal_length;
  jacobianOplusXj_(1, 3) = 0;
  jacobianOplusXj_(1, 4) = -1. / z * cam.focal_length;
  jacobianOplusXj_(1, 5) = y / z_2 * cam.focal_length;
}

}  // namespace g2o
