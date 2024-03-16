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

#include "edge_project_psi2uv.h"

#include <Eigen/Core>

#include "g2o/types/sba/parameter_cameraparameters.h"
#include "g2o/types/sba/vertex_se3_expmap.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "sba_utils.h"

namespace g2o {

EdgeProjectPSI2UV::EdgeProjectPSI2UV() {
  resizeParameters(1);
  installParameter<CameraParameters>(0);
}

void EdgeProjectPSI2UV::computeError() {
  const VertexPointXYZ* psi = vertexXnRaw<0>();
  const VertexSE3Expmap* T_p_from_world = vertexXnRaw<1>();
  const VertexSE3Expmap* T_anchor_from_world = vertexXnRaw<2>();
  const CameraParameters* cam =
      static_cast<const CameraParameters*>(parameter(0).get());

  Vector2 obs(measurement_);
  error_ = obs - cam->cam_map(T_p_from_world->estimate() *
                              T_anchor_from_world->estimate().inverse() *
                              internal::invert_depth(psi->estimate()));
}

void EdgeProjectPSI2UV::linearizeOplus() {
  VertexPointXYZ* vpoint = vertexXnRaw<0>();
  Vector3 psi_a = vpoint->estimate();
  VertexSE3Expmap* vpose = vertexXnRaw<1>();
  SE3Quat T_cw = vpose->estimate();
  VertexSE3Expmap* vanchor = vertexXnRaw<2>();
  const StereoCameraParameters& cam =
      static_cast<const CameraParameters*>(parameter(0).get())->param();

  SE3Quat A_aw = vanchor->estimate();
  SE3Quat T_ca = T_cw * A_aw.inverse();
  Vector3 x_a = internal::invert_depth(psi_a);
  Vector3 y = T_ca * x_a;
  Eigen::Matrix<double, 2, 3, Eigen::ColMajor> Jcam =
      internal::d_proj_d_y(cam.focal_length, y);

  auto& jacobianOplus0 = std::get<0>(this->jacobianOplus_);
  auto& jacobianOplus1 = std::get<1>(this->jacobianOplus_);
  auto& jacobianOplus2 = std::get<2>(this->jacobianOplus_);
  jacobianOplus0 = -Jcam * internal::d_Tinvpsi_d_psi(T_ca, psi_a);
  jacobianOplus1 = -Jcam * internal::d_expy_d_y(y);
  jacobianOplus2 =
      Jcam * T_ca.rotation().toRotationMatrix() * internal::d_expy_d_y(x_a);
}

}  // namespace g2o
