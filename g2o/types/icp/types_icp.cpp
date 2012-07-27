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
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {

  G2O_REGISTER_TYPE_GROUP(icp);
  G2O_REGISTER_TYPE(EDGE_V_V_GICP, Edge_V_V_GICP);

  namespace types_icp {
    int initialized = 0;

    void init()
    {
      if (types_icp::initialized)
        return;
      //cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;

      Edge_V_V_GICP::dRidx << 0.0,0.0,0.0,
        0.0,0.0,2.0,
        0.0,-2.0,0.0;
      Edge_V_V_GICP::dRidy  << 0.0,0.0,-2.0,
        0.0,0.0,0.0,
        2.0,0.0,0.0;
      Edge_V_V_GICP::dRidz  << 0.0,2.0,0.0,
        -2.0,0.0,0.0,
        0.0,0.0,0.0;

      VertexSCam::dRidx << 0.0,0.0,0.0,
        0.0,0.0,2.0,
        0.0,-2.0,0.0;
      VertexSCam::dRidy  << 0.0,0.0,-2.0,
        0.0,0.0,0.0,
        2.0,0.0,0.0;
      VertexSCam::dRidz  << 0.0,2.0,0.0,
        -2.0,0.0,0.0,
        0.0,0.0,0.0;

      types_icp::initialized = 1;
    }
  }

  using namespace std;
  using namespace Eigen;
  typedef  Matrix<double, 6, 1> Vector6d;

  Matrix3d Edge_V_V_GICP::dRidx; // differential quat matrices
  Matrix3d Edge_V_V_GICP::dRidy; // differential quat matrices
  Matrix3d Edge_V_V_GICP::dRidz; // differential quat matrices
  Matrix3d VertexSCam::dRidx; // differential quat matrices
  Matrix3d VertexSCam::dRidy; // differential quat matrices
  Matrix3d VertexSCam::dRidz; // differential quat matrices
  Matrix3d VertexSCam::Kcam;
  double VertexSCam::baseline;

  // global initialization
  G2O_ATTRIBUTE_CONSTRUCTOR(init_icp_types)
  {
    types_icp::init();
  }

  // Copy constructor
  Edge_V_V_GICP::Edge_V_V_GICP(const Edge_V_V_GICP* e)
    : BaseBinaryEdge<3, EdgeGICP, VertexSE3, VertexSE3>()
  {

    // Temporary hack - TODO, sort out const-ness properly
    _vertices[0] = const_cast<HyperGraph::Vertex*> (e->vertex(0));
    _vertices[1] = const_cast<HyperGraph::Vertex*> (e->vertex(1));

    _measurement.pos0 = e->measurement().pos0;
    _measurement.pos1 = e->measurement().pos1;
    _measurement.normal0 = e->measurement().normal0;
    _measurement.normal1 = e->measurement().normal1;
    _measurement.R0 = e->measurement().R0;
    _measurement.R1 = e->measurement().R1;

    pl_pl = e->pl_pl;
    cov0 = e->cov0;
    cov1 = e->cov1;

    // TODO the robust kernel is not correctly copied
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

  bool Edge_V_V_GICP::read(std::istream& is)
  {
    // measured point and normal
    for (int i=0; i<3; i++)
      is >> _measurement.pos0[i];
    for (int i=0; i<3; i++)
      is >> _measurement.normal0[i];

    // measured point and normal
    for (int i=0; i<3; i++)
      is >> _measurement.pos1[i];
    for (int i=0; i<3; i++)
      is >> _measurement.normal1[i];

    // don't need this if we don't use it in error calculation (???)
    //    inverseMeasurement() = -measurement();

    _measurement.makeRot0();  // set up rotation matrices

    // GICP info matrices

    // point-plane only
    Matrix3d prec;
    double v = .01;
    prec << v, 0, 0,
            0, v, 0,
            0, 0, 1;
    const Matrix3d &R = measurement().R0; // plane of the point in vp0
    information() = R.transpose()*prec*R;

    //    information().setIdentity();

    //    setRobustKernel(true);
    //setHuberWidth(0.01);      // units? m?

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
  void Edge_V_V_GICP::linearizeOplus()
  {
    VertexSE3* vp0 = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3* vp1 = static_cast<VertexSE3*>(_vertices[1]);

    // topLeftCorner<3,3>() is the rotation matrix
    Matrix3d R0T = vp0->estimate().matrix().topLeftCorner<3,3>().transpose();
    Vector3d p1 = measurement().pos1;

    // this could be more efficient
    if (!vp0->fixed())
      {
        Eigen::Isometry3d T01 = vp0->estimate().inverse() *  vp1->estimate();
        Vector3d p1t = T01 * p1;
        _jacobianOplusXi.block<3,3>(0,0) = -Matrix3d::Identity();
        _jacobianOplusXi.block<3,1>(0,3) = dRidx*p1t;
        _jacobianOplusXi.block<3,1>(0,4) = dRidy*p1t;
        _jacobianOplusXi.block<3,1>(0,5) = dRidz*p1t;
      }

    if (!vp1->fixed())
      {
        Matrix3d R1 = vp1->estimate().matrix().topLeftCorner<3,3>();
        R0T = R0T*R1;
        _jacobianOplusXj.block<3,3>(0,0) = R0T;
        _jacobianOplusXj.block<3,1>(0,3) = R0T*dRidx.transpose()*p1;
        _jacobianOplusXj.block<3,1>(0,4) = R0T*dRidy.transpose()*p1;
        _jacobianOplusXj.block<3,1>(0,5) = R0T*dRidz.transpose()*p1;
      }
  }
#endif


  bool Edge_V_V_GICP::write(std::ostream& os) const
  {
    // first point
    for (int i=0; i<3; i++)
      os  << measurement().pos0[i] << " ";
    for (int i=0; i<3; i++)
      os  << measurement().normal0[i] << " ";

    // second point
    for (int i=0; i<3; i++)
      os  << measurement().pos1[i] << " ";
    for (int i=0; i<3; i++)
      os  << measurement().normal1[i] << " ";


    return os.good();
  }

  //
  // stereo camera functions
  //



  VertexSCam::VertexSCam() :
    VertexSE3()
  {}


  Edge_XYZ_VSC::Edge_XYZ_VSC()
  {}

#ifdef SCAM_ANALYTIC_JACOBIANS
/**
 * \brief Jacobian for stereo projection
 */
  void Edge_XYZ_VSC::linearizeOplus()
  {
    VertexSCam *vc = static_cast<VertexSCam *>(_vertices[1]);

    VertexSBAPointXYZ *vp = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
    Vector4d pt, trans;
    pt.head<3>() = vp->estimate();
    pt(3) = 1.0;
    trans.head<3>() = vc->estimate().translation();
    trans(3) = 1.0;

    // first get the world point in camera coords
    Eigen::Matrix<double,3,1> pc = vc->w2n * pt;

    // Jacobians wrt camera parameters
    // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    double px = pc(0);
    double py = pc(1);
    double pz = pc(2);
    double ipz2 = 1.0/(pz*pz);
    if (isnan(ipz2) )
      {
  std::cout << "[SetJac] infinite jac" << std::endl;
  *(int *)0x0 = 0;
      }

    double ipz2fx = ipz2*vc->Kcam(0,0); // Fx
    double ipz2fy = ipz2*vc->Kcam(1,1); // Fy
    double b      = vc->baseline; // stereo baseline

    Eigen::Matrix<double,3,1> pwt;

    // check for local vars
    pwt = (pt-trans).head<3>(); // transform translations, use differential rotation

    // dx
    Eigen::Matrix<double,3,1> dp = vc->dRdx * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,3) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,3) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,3) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    // dy
    dp = vc->dRdy * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,4) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,4) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,4) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    // dz
    dp = vc->dRdz * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,5) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,5) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,5) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px

    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = -vc->w2n.col(0);        // dpc / dx
    _jacobianOplusXj(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,0) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = -vc->w2n.col(1);        // dpc / dy
    _jacobianOplusXj(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,1) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = -vc->w2n.col(2);        // dpc / dz
    _jacobianOplusXj(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,2) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px

    // Jacobians wrt point parameters
    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = vc->w2n.col(0); // dpc / dx
    _jacobianOplusXi(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,0) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = vc->w2n.col(1); // dpc / dy
    _jacobianOplusXi(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,1) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = vc->w2n.col(2); // dpc / dz
    _jacobianOplusXi(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,2) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
  }
#endif
  bool Edge_XYZ_VSC::read(std::istream&)
  { return false; }

  bool Edge_XYZ_VSC::write(std::ostream&) const
  { return false; }

  bool VertexSCam::read(std::istream&)
  { return false; }

  bool VertexSCam::write(std::ostream&) const
  { return false; }

} // end namespace
