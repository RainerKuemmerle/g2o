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

#include "types_icp.h"

#include <iostream>

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

namespace g2o {

G2O_REGISTER_TYPE_GROUP(icp);
G2O_REGISTER_TYPE(EDGE_V_V_GICP, EdgeVVGicp);

namespace types_icp {
int initialized = 0;

void init() {
  if (types_icp::initialized) return;
  // cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;

  EdgeVVGicp::dRidx_ << 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0;
  EdgeVVGicp::dRidy_ << 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0;
  EdgeVVGicp::dRidz_ << 0.0, 2.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  VertexSCam::dRidx_ << 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0;
  VertexSCam::dRidy_ << 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0;
  VertexSCam::dRidz_ << 0.0, 2.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  types_icp::initialized = 1;
}
}  // namespace types_icp

Matrix3 EdgeVVGicp::dRidx_;  // differential quat matrices
Matrix3 EdgeVVGicp::dRidy_;  // differential quat matrices
Matrix3 EdgeVVGicp::dRidz_;  // differential quat matrices
Matrix3 VertexSCam::dRidx_;  // differential quat matrices
Matrix3 VertexSCam::dRidy_;  // differential quat matrices
Matrix3 VertexSCam::dRidz_;  // differential quat matrices
Matrix3 VertexSCam::kcam_;
number_t VertexSCam::baseline_;

// global initialization
G2O_ATTRIBUTE_CONSTRUCTOR(init_icp_types) { types_icp::init(); }

// Copy constructor
EdgeVVGicp::EdgeVVGicp(const EdgeVVGicp *e)

{
  // Temporary hack - TODO, sort out const-ness properly
  vertices_[0] = std::const_pointer_cast<HyperGraph::Vertex>(e->vertex(0));
  vertices_[1] = std::const_pointer_cast<HyperGraph::Vertex>(e->vertex(1));

  measurement_.pos0 = e->measurement().pos0;
  measurement_.pos1 = e->measurement().pos1;
  measurement_.normal0 = e->measurement().normal0;
  measurement_.normal1 = e->measurement().normal1;
  measurement_.R0 = e->measurement().R0;
  measurement_.R1 = e->measurement().R1;

  pl_pl = e->pl_pl;
  cov0 = e->cov0;
  cov1 = e->cov1;

  // TODO(goki): the robust kernel is not correctly copied
  //_robustKernel = e->_robustKernel;
}

//
// Rigid 3D constraint between poses, given fixed point offsets
//

// input two matched points between the frames
// first point belongs to the first frame, position and normal
// second point belongs to the second frame, position and normal
//
// the measurement variable has type EdgeGICP (see types_icp.h)

bool EdgeVVGicp::read(std::istream &is) {
  // measured point and normal
  for (int i = 0; i < 3; i++) is >> measurement_.pos0[i];
  for (int i = 0; i < 3; i++) is >> measurement_.normal0[i];

  // measured point and normal
  for (int i = 0; i < 3; i++) is >> measurement_.pos1[i];
  for (int i = 0; i < 3; i++) is >> measurement_.normal1[i];

  // don't need this if we don't use it in error calculation (???)
  //    inverseMeasurement() = -measurement();

  measurement_.makeRot0();  // set up rotation matrices

  // GICP info matrices

  // point-plane only
  Matrix3 prec;
  number_t v = cst(.01);
  prec << v, 0, 0, 0, v, 0, 0, 0, 1;
  const Matrix3 &R = measurement().R0;  // plane of the point in vp0
  information() = R.transpose() * prec * R;

  //    information().setIdentity();

  //    setRobustKernel(true);
  // setHuberWidth(0.01);      // units? m?

  return true;
}

// Jacobian
// [ -R0'*R1 | R0 * dRdx/ddx * 0p1 ]
// [  R0'*R1 | R0 * dR'dx/ddx * 0p1 ]

#ifdef GICP_ANALYTIC_JACOBIANS

// jacobian defined as:
//    f(T0,T1) =  dR0.inv() * T0.inv() * (T1 * dR1 * p1 + dt1) - dt0
//    df/dx0 = [-I, d[dR0.inv()]/dq0 * T01 * p1]
//    df/dx1 = [R0, T01 * d[dR1]/dq1 * p1]
void EdgeVVGicp::linearizeOplus() {
  VertexSE3 *vp0 = vertexXnRaw<0>();
  VertexSE3 *vp1 = vertexXnRaw<1>();

  // topLeftCorner<3,3>() is the rotation matrix
  Matrix3 R0T = vp0->estimate().matrix().topLeftCorner<3, 3>().transpose();
  Vector3 p1 = measurement().pos1;

  // this could be more efficient
  if (!vp0->fixed()) {
    Isometry3 T01 = vp0->estimate().inverse() * vp1->estimate();
    Vector3 p1t = T01 * p1;
    jacobianOplusXi_.block<3, 3>(0, 0) = -Matrix3::Identity();
    jacobianOplusXi_.block<3, 1>(0, 3) = dRidx_ * p1t;
    jacobianOplusXi_.block<3, 1>(0, 4) = dRidy_ * p1t;
    jacobianOplusXi_.block<3, 1>(0, 5) = dRidz_ * p1t;
  }

  if (!vp1->fixed()) {
    Matrix3 R1 = vp1->estimate().matrix().topLeftCorner<3, 3>();
    R0T = R0T * R1;
    jacobianOplusXj_.block<3, 3>(0, 0) = R0T;
    jacobianOplusXj_.block<3, 1>(0, 3) = R0T * dRidx_.transpose() * p1;
    jacobianOplusXj_.block<3, 1>(0, 4) = R0T * dRidy_.transpose() * p1;
    jacobianOplusXj_.block<3, 1>(0, 5) = R0T * dRidz_.transpose() * p1;
  }
}
#endif

bool EdgeVVGicp::write(std::ostream &os) const {
  // first point
  for (int i = 0; i < 3; i++) os << measurement().pos0[i] << " ";
  for (int i = 0; i < 3; i++) os << measurement().normal0[i] << " ";

  // second point
  for (int i = 0; i < 3; i++) os << measurement().pos1[i] << " ";
  for (int i = 0; i < 3; i++) os << measurement().normal1[i] << " ";

  return os.good();
}

//
// stereo camera functions
//

#ifdef SCAM_ANALYTIC_JACOBIANS
/**
 * \brief Jacobian for stereo projection
 */
void Edge_XYZ_VSC::linearizeOplus() {
  VertexSCam *vc = static_cast<VertexSCam *>(vertices_[1]);

  VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(vertices_[0]);
  Vector4 pt, trans;
  pt.head<3>() = vp->estimate();
  pt(3) = 1.0;
  trans.head<3>() = vc->estimate().translation();
  trans(3) = 1.0;

  // first get the world point in camera coords
  Eigen::Matrix<number_t, 3, 1, Eigen::ColMajor> pc = vc->w2n * pt;

  // Jacobians wrt camera parameters
  // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  number_t px = pc(0);
  number_t py = pc(1);
  number_t pz = pc(2);
  number_t ipz2 = 1.0 / (pz * pz);
  if (isnan(ipz2)) {
    std::cout << "[SetJac] infinite jac" << std::endl;
    *(int *)0x0 = 0;
  }

  number_t ipz2fx = ipz2 * vc->Kcam(0, 0);  // Fx
  number_t ipz2fy = ipz2 * vc->Kcam(1, 1);  // Fy
  number_t b = vc->baseline;                // stereo baseline

  Eigen::Matrix<number_t, 3, 1, Eigen::ColMajor> pwt;

  // check for local vars
  pwt = (pt - trans)
            .head<3>();  // transform translations, use differential rotation

  // dx
  Eigen::Matrix<number_t, 3, 1, Eigen::ColMajor> dp =
      vc->dRdx * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 3) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 3) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 3) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  // dy
  dp = vc->dRdy * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 4) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 4) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 4) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  // dz
  dp = vc->dRdz * pwt;  // dR'/dq * [pw - t]
  jacobianOplusXj_(0, 5) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 5) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 5) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px

  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = -vc->w2n.col(0);  // dpc / dx
  jacobianOplusXj_(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 0) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = -vc->w2n.col(1);                          // dpc / dy
  jacobianOplusXj_(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 1) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = -vc->w2n.col(2);                          // dpc / dz
  jacobianOplusXj_(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXj_(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXj_(2, 2) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px

  // Jacobians wrt point parameters
  // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
  dp = vc->w2n.col(0);  // dpc / dx
  jacobianOplusXi_(0, 0) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 0) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 0) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = vc->w2n.col(1);                           // dpc / dy
  jacobianOplusXi_(0, 1) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 1) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 1) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
  dp = vc->w2n.col(2);                           // dpc / dz
  jacobianOplusXi_(0, 2) = (pz * dp(0) - px * dp(2)) * ipz2fx;
  jacobianOplusXi_(1, 2) = (pz * dp(1) - py * dp(2)) * ipz2fy;
  jacobianOplusXi_(2, 2) =
      (pz * dp(0) - (px - b) * dp(2)) * ipz2fx;  // right image px
}
#endif
bool EdgeXyzVsc::read(std::istream &) { return false; }

bool EdgeXyzVsc::write(std::ostream &) const { return false; }

bool VertexSCam::read(std::istream &) { return false; }

bool VertexSCam::write(std::ostream &) const { return false; }

}  // namespace g2o
