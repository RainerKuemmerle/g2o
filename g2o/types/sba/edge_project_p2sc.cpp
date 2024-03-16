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

#include "edge_project_p2sc.h"

#include <cmath>

#include "g2o/stuff/logger.h"
#include "g2o/stuff/misc.h"
#include "g2o/types/sba/sbacam.h"
#include "g2o/types/sba/vertex_cam.h"

namespace g2o {

// return the error estimate as a 2-vector
void EdgeProjectP2SC::computeError() {
  // from <Point> to <Cam>
  const VertexPointXYZ* point = vertexXnRaw<0>();
  VertexCam* cam = vertexXnRaw<1>();

  // calculate the projection
  Vector3 kp;
  Vector4 pt;
  pt.head<3>() = point->estimate();
  pt(3) = 1;
  const SBACam& nd = cam->estimate();
  // these should be already ok
  /* nd.setTransform(); */
  /* nd.setProjection(); */
  /* nd.setDr(); */

  Vector3 p1 = nd.w2i * pt;
  Vector3 p2 = nd.w2n * pt;
  Vector3 pb(nd.baseline, 0, 0);

  double invp1 = cst(1.0) / p1(2);
  kp.head<2>() = p1.head<2>() * invp1;

  // right camera px
  p2 = nd.Kcam * (p2 - pb);
  kp(2) = p2(0) / p2(2);

  // std::cout << std::endl << "CAM   " << cam->estimate() << std::endl;
  // std::cout << "POINT " << pt.transpose() << std::endl;
  // std::cout << "PROJ  " << p1.transpose() << std::endl;
  // std::cout << "PROJ  " << p2.transpose() << std::endl;
  // std::cout << "CPROJ " << kp.transpose() << std::endl;
  // std::cout << "MEAS  " << measurement_.transpose() << std::endl;

  // error, which is backwards from the normal observed - calculated
  // measurement_ is the measured projection
  error_ = kp - measurement_;
}

void EdgeProjectP2SC::linearizeOplus() {
  VertexCam* vc = vertexXnRaw<1>();
  const SBACam& cam = vc->estimate();

  VertexPointXYZ* vp = vertexXnRaw<0>();
  Vector4 pt;
  Vector4 trans;
  pt.head<3>() = vp->estimate();
  pt(3) = 1.0;
  trans.head<3>() = cam.translation();
  trans(3) = 1.0;

  // first get the world point in camera coords
  Vector3 pc = cam.w2n * pt;

  // Jacobians wrt camera parameters
  // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  double px = pc(0);
  double py = pc(1);
  double pz = pc(2);
  double ipz2 = 1.0 / (pz * pz);
  if (std::isnan(ipz2)) {
    G2O_ERROR("[SetJac] infinite jac");
    abort();
  }

  double ipz2fx = ipz2 * cam.Kcam(0, 0);  // Fx
  double ipz2fy = ipz2 * cam.Kcam(1, 1);  // Fy
  double b = cam.baseline;                // stereo baseline

  // check for local vars
  Vector3 pwt =
      (pt - trans)
          .head<3>();  // transform translations, use differential rotation

  // dx
  Vector3 dp = cam.dRdx * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 3) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 3) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 3) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  // dy
  dp = cam.dRdy * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 4) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 4) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 4) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  // dz
  dp = cam.dRdz * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 5) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 5) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 5) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px

  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = -cam.w2n.col(0);  // dpc / dx
  jacobianOplusXj_(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 0) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = -cam.w2n.col(1);                          // dpc / dy
  jacobianOplusXj_(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 1) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = -cam.w2n.col(2);                          // dpc / dz
  jacobianOplusXj_(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 2) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px

  // Jacobians wrt point parameters
  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = cam.w2n.col(0);  // dpc / dx
  jacobianOplusXi_(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 0) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = cam.w2n.col(1);                           // dpc / dy
  jacobianOplusXi_(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 1) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = cam.w2n.col(2);                           // dpc / dz
  jacobianOplusXi_(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 2) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
}

}  // namespace g2o
