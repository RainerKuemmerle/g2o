// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#include "edge_project_p2mc.h"

namespace g2o {

// point to camera projection, monocular
EdgeProjectP2MC::EdgeProjectP2MC()
    : BaseBinaryEdge<2, Vector2, VertexPointXYZ, VertexCam>() {
  information().setIdentity();
}

bool EdgeProjectP2MC::read(std::istream &is) {
  // measured keypoint
  internal::readVector(is, _measurement);
  return readInformationMatrix(is);
}

bool EdgeProjectP2MC::write(std::ostream &os) const {
  internal::writeVector(os, measurement());
  writeInformationMatrix(os);
  return os.good();
}

void EdgeProjectP2MC::computeError() {
  // from <Point> to <Cam>
  const VertexPointXYZ *point =
      static_cast<const VertexPointXYZ *>(_vertices[0]);
  const VertexCam *cam = static_cast<const VertexCam *>(_vertices[1]);

  // calculate the projection
  const Vector3 &pt = point->estimate();
  Vector4 ppt(pt(0), pt(1), pt(2), 1);
  Vector3 p = cam->estimate().w2i * ppt;
  Vector2 perr;
  perr = p.head<2>() / p(2);
  //      std::cout << std::endl << "CAM   " << cam->estimate() << std::endl;
  //      std::cout << "POINT " << pt.transpose() << std::endl;
  //      std::cout << "PROJ  " << p.transpose() << std::endl;
  //      std::cout << "CPROJ " << perr.transpose() << std::endl;
  //      std::cout << "MEAS  " << _measurement.transpose() << std::endl;

  // error, which is backwards from the normal observed - calculated
  // _measurement is the measured projection
  _error = perr - _measurement;
  // std::cerr << _error.x() << " " << _error.y() <<  " " << chi2() <<
  // std::endl;
}

void EdgeProjectP2MC::linearizeOplus() {
  VertexCam *vc = static_cast<VertexCam *>(_vertices[1]);
  const SBACam &cam = vc->estimate();

  VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(_vertices[0]);
  Vector4 pt, trans;
  pt.head<3>() = vp->estimate();
  pt(3) = 1.0;
  trans.head<3>() = cam.translation();
  trans(3) = 1.0;

  // first get the world point in camera coords
  Vector3 pc = cam.w2n * pt;

  // Jacobians wrt camera parameters
  // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  number_t px = pc(0);
  number_t py = pc(1);
  number_t pz = pc(2);
  number_t ipz2 = 1.0 / (pz * pz);
  if (g2o_isnan(ipz2)) {
    std::cout << "[SetJac] infinite jac" << std::endl;
    abort();
  }

  number_t ipz2fx = ipz2 * cam.Kcam(0, 0);  // Fx
  number_t ipz2fy = ipz2 * cam.Kcam(1, 1);  // Fy

  // check for local vars
  Vector3 pwt =
      (pt - trans)
          .head<3>();  // transform translations, use differential rotation

  // dx
  Vector3 dp = cam.dRdx * pwt;  // dR'/dq * [pw - t]
  _jacobianOplusXj(0, 3) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXj(1, 3) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  // dy
  dp = cam.dRdy * pwt;  // dR'/dq * [pw - t]
  _jacobianOplusXj(0, 4) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXj(1, 4) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  // dz
  dp = cam.dRdz * pwt;  // dR'/dq * [pw - t]
  _jacobianOplusXj(0, 5) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXj(1, 5) = (pz * dp(1) - py * dp(2)) * ipz2fy;

  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = -cam.w2n.col(0);  // dpc / dx
  _jacobianOplusXj(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXj(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  dp = -cam.w2n.col(1);  // dpc / dy
  _jacobianOplusXj(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXj(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  dp = -cam.w2n.col(2);  // dpc / dz
  _jacobianOplusXj(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXj(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;

  // Jacobians wrt point parameters
  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = cam.w2n.col(0);  // dpc / dx
  _jacobianOplusXi(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXi(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  dp = cam.w2n.col(1);  // dpc / dy
  _jacobianOplusXi(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXi(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  dp = cam.w2n.col(2);  // dpc / dz
  _jacobianOplusXi(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  _jacobianOplusXi(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;
}

}  // namespace g2o
