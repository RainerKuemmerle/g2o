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

#include "g2o/stuff/logger.h"
#include "g2o/types/sba/sbacam.h"
#include "g2o/types/sba/vertex_cam.h"

namespace g2o {

// point to camera projection, monocular
EdgeProjectP2MC::EdgeProjectP2MC() { information().setIdentity(); }

void EdgeProjectP2MC::computeError() {
  // from <Point> to <Cam>
  const VertexPointXYZ* point = vertexXnRaw<0>();
  const VertexCam* cam = vertexXnRaw<1>();

  // calculate the projection
  const Vector3& pt = point->estimate();
  Vector4 ppt(pt(0), pt(1), pt(2), 1);
  Vector3 p = cam->estimate().w2i * ppt;
  Vector2 perr;
  perr = p.head<2>() / p(2);

  // error, which is backwards from the normal observed - calculated
  // measurement_ is the measured projection
  error_ = perr - measurement_;
}

void EdgeProjectP2MC::linearizeOplus() {
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
    G2O_CRITICAL("[SetJac] infinite jac");
    abort();
  }

  double ipz2fx = ipz2 * cam.Kcam(0, 0);  // Fx
  double ipz2fy = ipz2 * cam.Kcam(1, 1);  // Fy

  // check for local vars
  Vector3 pwt =
      (pt - trans)
          .head<3>();  // transform translations, use differential rotation

  // dx
  Vector3 dp = cam.dRdx * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 3) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 3) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  // dy
  dp = cam.dRdy * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 4) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 4) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  // dz
  dp = cam.dRdz * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 5) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 5) = (pz * dp(1) - py * dp(2)) * ipz2fy;

  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = -cam.w2n.col(0);  // dpc / dx
  jacobianOplusXj_(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  dp = -cam.w2n.col(1);  // dpc / dy
  jacobianOplusXj_(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  dp = -cam.w2n.col(2);  // dpc / dz
  jacobianOplusXj_(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;

  // Jacobians wrt point parameters
  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = cam.w2n.col(0);  // dpc / dx
  jacobianOplusXi_(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  dp = cam.w2n.col(1);  // dpc / dy
  jacobianOplusXi_(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  dp = cam.w2n.col(2);  // dpc / dz
  jacobianOplusXi_(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;
}

}  // namespace g2o
