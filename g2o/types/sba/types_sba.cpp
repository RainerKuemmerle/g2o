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

#include "types_sba.h"
#include <iostream>

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

namespace g2o {

  using namespace std;

  G2O_REGISTER_TYPE_GROUP(sba);

  G2O_REGISTER_TYPE(VERTEX_CAM, VertexCam);
  G2O_REGISTER_TYPE(VERTEX_XYZ, VertexSBAPointXYZ);
  G2O_REGISTER_TYPE(VERTEX_INTRINSICS, VertexIntrinsics);

  G2O_REGISTER_TYPE(EDGE_PROJECT_P2MC, EdgeProjectP2MC);
  G2O_REGISTER_TYPE(EDGE_PROJECT_P2SC, EdgeProjectP2SC);
  G2O_REGISTER_TYPE(EDGE_CAM, EdgeSBACam);
  G2O_REGISTER_TYPE(EDGE_SCALE, EdgeSBAScale);

  // constructor
  VertexIntrinsics::VertexIntrinsics()
  {
    _estimate << cst(1.), cst(1.), cst(.5), cst(.5), cst(.1);
  }

  bool VertexIntrinsics::read(std::istream& is) { return internal::readVector(is, _estimate); }

  bool VertexIntrinsics::write(std::ostream& os) const { return internal::writeVector(os, estimate()); }

  // constructor
  VertexCam::VertexCam()
  {
  }

  bool VertexCam::read(std::istream& is)
  {
    // first the position and orientation (vector3 and quaternion)
    Vector3 t;
    internal::readVector(is, t);
    Quaternion r;
    internal::readVector(is, r.coeffs());
    r.normalize(); // recover nummeric precision

    // form the camera object
    SBACam cam(r,t);

    // now fx, fy, cx, cy, baseline
    number_t fx, fy, cx, cy, tx;

    // try to read one value
    is >>  fx;
    if (is.good()) {
      is >>  fy >> cx >> cy >> tx;
      cam.setKcam(fx,fy,cx,cy,tx);
    } else{
      is.clear();
      std::cerr << "cam not defined, using defaults" << std::endl;
      cam.setKcam(300,300,320,320,cst(0.1));
    }

    setEstimate(cam);
    return true;
  }

  bool VertexCam::write(std::ostream& os) const
  {
    const SBACam &cam = estimate();

    // first the position and orientation (vector3 and quaternion)
    internal::writeVector(os, cam.translation());
    internal::writeVector(os, cam.rotation().coeffs());

    // now fx, fy, cx, cy, baseline
    os << cam.Kcam(0,0) << " ";
    os << cam.Kcam(1,1) << " ";
    os << cam.Kcam(0,2) << " ";
    os << cam.Kcam(1,2) << " ";
    os << cam.baseline << " ";
    return os.good();
  }

  EdgeSBACam::EdgeSBACam() :
    BaseBinaryEdge<6, SE3Quat, VertexCam, VertexCam>()
  {
  }

  bool EdgeSBACam::read(std::istream& is)
  {
    Vector7 meas;
    internal::readVector(is, meas);
    setMeasurement(SE3Quat(meas));
    return readInformationMatrix(is);
  }

  bool EdgeSBACam::write(std::ostream& os) const
  {
    internal::writeVector(os, measurement().toVector());
    return writeInformationMatrix(os);
  }

  void EdgeSBACam::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
  {
    VertexCam* from = static_cast<VertexCam*>(_vertices[0]);
    VertexCam* to = static_cast<VertexCam*>(_vertices[1]);
    if (from_.count(from) > 0)
      to->setEstimate((SE3Quat) from->estimate() * _measurement);
    else
      from->setEstimate((SE3Quat) to->estimate() * _inverseMeasurement);
  }


  VertexSBAPointXYZ::VertexSBAPointXYZ() : BaseVertex<3, Vector3>()
  {
  }

  bool VertexSBAPointXYZ::read(std::istream& is)
  {
    return internal::readVector(is, _estimate);
  }

  bool VertexSBAPointXYZ::write(std::ostream& os) const
  {
    return internal::writeVector(os, estimate());
  }

  // point to camera projection, monocular
  EdgeProjectP2MC::EdgeProjectP2MC() :
  BaseBinaryEdge<2, Vector2, VertexSBAPointXYZ, VertexCam>()
  {
    information().setIdentity();
  }

  bool EdgeProjectP2MC::read(std::istream& is)
  {
    // measured keypoint
    internal::readVector(is, _measurement);
    return readInformationMatrix(is);
  }

  bool EdgeProjectP2MC::write(std::ostream& os) const
  {
    internal::writeVector(os, measurement());
    writeInformationMatrix(os);
    return os.good();
  }

  // point to camera projection, stereo
  EdgeProjectP2SC::EdgeProjectP2SC() :
    BaseBinaryEdge<3, Vector3, VertexSBAPointXYZ, VertexCam>()
  {
  }

  bool EdgeProjectP2SC::read(std::istream& is)
  {
    internal::readVector(is, _measurement);
    return readInformationMatrix(is);
  }

  bool EdgeProjectP2SC::write(std::ostream& os) const
  {
    internal::writeVector(os, measurement());
    writeInformationMatrix(os);
    return os.good();
  }

/**
 * \brief Jacobian for stereo projection
 */
  void EdgeProjectP2SC::linearizeOplus()
  {
    VertexCam *vc = static_cast<VertexCam *>(_vertices[1]);
    const SBACam &cam = vc->estimate();

    VertexSBAPointXYZ *vp = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
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
    number_t ipz2 = 1.0/(pz*pz);
    if (g2o_isnan(ipz2) ) {
      std::cout << "[SetJac] infinite jac" << std::endl;
      abort();
    }

    number_t ipz2fx = ipz2*cam.Kcam(0,0); // Fx
    number_t ipz2fy = ipz2*cam.Kcam(1,1); // Fy
    number_t b      = cam.baseline; // stereo baseline

    // check for local vars
    Vector3 pwt = (pt-trans).head<3>(); // transform translations, use differential rotation

    // dx
    Vector3 dp = cam.dRdx * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,3) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,3) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,3) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    // dy
    dp = cam.dRdy * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,4) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,4) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,4) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    // dz
    dp = cam.dRdz * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,5) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,5) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,5) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px

    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = -cam.w2n.col(0);        // dpc / dx
    _jacobianOplusXj(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,0) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = -cam.w2n.col(1);        // dpc / dy
    _jacobianOplusXj(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,1) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = -cam.w2n.col(2);        // dpc / dz
    _jacobianOplusXj(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXj(2,2) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px

    // Jacobians wrt point parameters
    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = cam.w2n.col(0); // dpc / dx
    _jacobianOplusXi(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,0) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = cam.w2n.col(1); // dpc / dy
    _jacobianOplusXi(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,1) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
    dp = cam.w2n.col(2); // dpc / dz
    _jacobianOplusXi(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
    _jacobianOplusXi(2,2) = (pz*dp(0) - (px-b)*dp(2))*ipz2fx; // right image px
  }


/**
 * \brief Jacobian for monocular projection
 */
  void EdgeProjectP2MC::linearizeOplus()
  {
    VertexCam *vc = static_cast<VertexCam *>(_vertices[1]);
    const SBACam &cam = vc->estimate();

    VertexSBAPointXYZ *vp = static_cast<VertexSBAPointXYZ *>(_vertices[0]);
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
    number_t ipz2 = 1.0/(pz*pz);
    if (g2o_isnan(ipz2) ) {
      std::cout << "[SetJac] infinite jac" << std::endl;
      abort();
    }

    number_t ipz2fx = ipz2*cam.Kcam(0,0); // Fx
    number_t ipz2fy = ipz2*cam.Kcam(1,1); // Fy

    // check for local vars
    Vector3 pwt = (pt-trans).head<3>(); // transform translations, use differential rotation

    // dx
    Vector3 dp = cam.dRdx * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,3) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,3) = (pz*dp(1) - py*dp(2))*ipz2fy;
    // dy
    dp = cam.dRdy * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,4) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,4) = (pz*dp(1) - py*dp(2))*ipz2fy;
    // dz
    dp = cam.dRdz * pwt; // dR'/dq * [pw - t]
    _jacobianOplusXj(0,5) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,5) = (pz*dp(1) - py*dp(2))*ipz2fy;

    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = -cam.w2n.col(0);        // dpc / dx
    _jacobianOplusXj(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = -cam.w2n.col(1);        // dpc / dy
    _jacobianOplusXj(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = -cam.w2n.col(2);        // dpc / dz
    _jacobianOplusXj(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXj(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;

    // Jacobians wrt point parameters
    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = cam.w2n.col(0); // dpc / dx
    _jacobianOplusXi(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = cam.w2n.col(1); // dpc / dy
    _jacobianOplusXi(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = cam.w2n.col(2); // dpc / dz
    _jacobianOplusXi(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    _jacobianOplusXi(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
  }

  // point to camera projection, stereo
  EdgeSBAScale::EdgeSBAScale() :
    BaseBinaryEdge<1, number_t, VertexCam, VertexCam>()
  {
  }

  bool EdgeSBAScale::read(std::istream& is)
  {
    number_t meas;
    is >> meas;
    setMeasurement(meas);
    information().setIdentity();
    is >> information()(0,0);
    return true;
  }

  bool EdgeSBAScale::write(std::ostream& os) const
  {
    os  << measurement() << " " << information()(0,0);
    return os.good();
  }

  void EdgeSBAScale::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/)
  {
    VertexCam* v1 = dynamic_cast<VertexCam*>(_vertices[0]);
    VertexCam* v2 = dynamic_cast<VertexCam*>(_vertices[1]);
    //compute the translation vector of v1 w.r.t v2
    if (from_.count(v1) == 1){
      SE3Quat delta = (v1->estimate().inverse()*v2->estimate());
      number_t norm =  delta.translation().norm();
      number_t alpha = _measurement/norm;
      delta.setTranslation(delta.translation()*alpha);
      v2->setEstimate(v1->estimate()*delta);
    } else {
      SE3Quat delta = (v2->estimate().inverse()*v1->estimate());
      number_t norm =  delta.translation().norm();
      number_t alpha = _measurement/norm;
      delta.setTranslation(delta.translation()*alpha);
      v1->setEstimate(v2->estimate()*delta);
    }
  }

  bool EdgeSBACam::setMeasurementFromState()
  {
    const VertexCam* v1 = dynamic_cast<const VertexCam*>(_vertices[0]);
    const VertexCam* v2 = dynamic_cast<const VertexCam*>(_vertices[1]);
    _measurement = (v1->estimate().inverse()*v2->estimate());
    _inverseMeasurement = _measurement.inverse();
    return true;
  }

} // end namespace
