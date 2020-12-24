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

#include "sba_utils.h"

namespace g2o {

EdgeProjectPSI2UV::EdgeProjectPSI2UV() {
  resizeParameters(1);
  installParameter(_cam, 0);
}

bool EdgeProjectPSI2UV::write(std::ostream &os) const {
  writeParamIds(os);
  internal::writeVector(os, measurement());
  return writeInformationMatrix(os);
}

bool EdgeProjectPSI2UV::read(std::istream &is) {
  readParamIds(is);
  internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

void EdgeProjectPSI2UV::computeError() {
  const VertexPointXYZ *psi = vertexXn<0>();
  const VertexSE3Expmap *T_p_from_world = vertexXn<1>();
  const VertexSE3Expmap *T_anchor_from_world = vertexXn<2>();
  const CameraParameters *cam = static_cast<const CameraParameters *>(parameter(0));

  Vector2 obs(_measurement);
  _error =
      obs - cam->cam_map(T_p_from_world->estimate() * T_anchor_from_world->estimate().inverse() *
                         internal::invert_depth(psi->estimate()));
}

void EdgeProjectPSI2UV::linearizeOplus() {
  VertexPointXYZ *vpoint = vertexXn<0>();
  Vector3 psi_a = vpoint->estimate();
  VertexSE3Expmap *vpose = vertexXn<1>();
  SE3Quat T_cw = vpose->estimate();
  VertexSE3Expmap *vanchor = vertexXn<2>();
  const CameraParameters *cam = static_cast<const CameraParameters *>(parameter(0));

  SE3Quat A_aw = vanchor->estimate();
  SE3Quat T_ca = T_cw * A_aw.inverse();
  Vector3 x_a = internal::invert_depth(psi_a);
  Vector3 y = T_ca * x_a;
  Eigen::Matrix<number_t, 2, 3, Eigen::ColMajor> Jcam = internal::d_proj_d_y(cam->focal_length, y);

  auto& jacobianOplus0 = std::get<0>(this->_jacobianOplus);
  auto& jacobianOplus1 = std::get<1>(this->_jacobianOplus);
  auto& jacobianOplus2 = std::get<2>(this->_jacobianOplus);
  jacobianOplus0 = -Jcam * internal::d_Tinvpsi_d_psi(T_ca, psi_a);
  jacobianOplus1 = -Jcam * internal::d_expy_d_y(y);
  jacobianOplus2 = Jcam * T_ca.rotation().toRotationMatrix() * internal::d_expy_d_y(x_a);
}

}  // namespace g2o
