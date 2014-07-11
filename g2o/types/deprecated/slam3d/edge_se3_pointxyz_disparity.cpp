// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include "edge_se3_pointxyz_disparity.h"

#include <iostream>
#include <iomanip>

#include "g2o/stuff/opengl_wrapper.h"

namespace g2o {
namespace deprecated {

  using namespace std;


  // point to camera projection, monocular
  EdgeSE3PointXYZDisparity::EdgeSE3PointXYZDisparity() : BaseBinaryEdge<3, Eigen::Vector3d, VertexSE3, VertexPointXYZ>() {
    resizeParameters(1);
    installParameter(params, 0);
    information().setIdentity();
    information()(2,2)=1000.;
    J.fill(0);
    J.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
  }


  bool EdgeSE3PointXYZDisparity::resolveCaches(){
    ParameterVector pv(1);
    pv[0]=params;
    resolveCache(cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_CAMERA",pv);
    return cache != 0;
  }


  bool EdgeSE3PointXYZDisparity::read(std::istream& is) {
    // measured keypoint
    int pid;
    is >> pid;
    setParameterId(0,pid);

    Eigen::Vector3d meas;
    for (int i=0; i<3; i++) is >> meas[i];
    setMeasurement(meas);
    if (is.bad())
      return false;
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix
      information().setIdentity();
      information()(2,2)=1000.;
    }
    //_cacheIds[0] = _paramId;
    return true;
  }

  bool EdgeSE3PointXYZDisparity::write(std::ostream& os) const {
    os << params->id() << " ";
    for (int i=0; i<3; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3PointXYZDisparity::computeError() {
    //VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = static_cast<VertexPointXYZ*>(_vertices[1]);
    const Eigen::Vector3d& pt = point->estimate();
    //Eigen::Vector4d ppt(pt(0),pt(1),pt(2),1.0);
    
    // VertexCameraCache* vcache = (VertexCameraCache*)cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }

    // CacheCamera* vcache = cache;
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }
    
    Eigen::Vector3d p = cache->w2i() * pt;

    Eigen::Vector3d perr;
    perr.head<2>() = p.head<2>()/p(2);
    perr(2) = 1/p(2);
    
    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _error = perr - _measurement;
  }

#ifdef EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN

  void EdgeSE3PointXYZDisparity::linearizeOplus() {
    //VertexSE3 *cam = static_cast<VertexSE3 *>(_vertices[0]);
    VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(_vertices[1]);

    // VertexCameraCache* vcache = (VertexCameraCache*)cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }

    // CacheCamera* vcache = cache;
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }


    const Eigen::Vector3d& pt = vp->estimate();

    Eigen::Vector3d Zcam = cache->w2lMatrix() * vp->estimate();

    //  J(0,3) = -0.0;
    J(0,4) = -2*Zcam(2);
    J(0,5) = 2*Zcam(1);

    J(1,3) = 2*Zcam(2);
    //  J(1,4) = -0.0;
    J(1,5) = -2*Zcam(0);

    J(2,3) = -2*Zcam(1);
    J(2,4) = 2*Zcam(0);
    //  J(2,5) = -0.0;

    J.block<3,3>(0,6) = cache->w2lMatrix().rotation();

    //Eigen::Matrix<double,3,9> Jprime = vcache->params->Kcam_inverseOffsetR  * J;
    Eigen::Matrix<double,3,9> Jprime = params->Kcam_inverseOffsetR()  * J;
    Eigen::Matrix<double, 3, 9> Jhom;
    Eigen::Vector3d Zprime = cache->w2i() * pt;

    Jhom.block<2,9>(0,0) = 1/(Zprime(2)*Zprime(2)) * (Jprime.block<2,9>(0,0)*Zprime(2) - Zprime.head<2>() * Jprime.block<1,9>(2,0));
    Jhom.block<1,9>(2,0) = - 1/(Zprime(2)*Zprime(2)) * Jprime.block<1,9>(2,0);

    _jacobianOplusXi = Jhom.block<3,6>(0,0);
    _jacobianOplusXj = Jhom.block<3,3>(0,6);
  }

#endif

  bool EdgeSE3PointXYZDisparity::setMeasurementFromState(){
    //VertexSE3 *cam = static_cast< VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = static_cast<VertexPointXYZ*>(_vertices[1]);
    const Eigen::Vector3d &pt = point->estimate();

    // VertexCameraCache* vcache = (VertexCameraCache*) cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }

    Eigen::Vector3d p = cache->w2i() * pt;

    Eigen::Vector3d perr;
    perr.head<2>() = p.head<2>()/p(2);
    perr(2) = 1/p(2);

    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _measurement = perr;
    return true;
  }

  void EdgeSE3PointXYZDisparity::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/)
  {
    (void) from;
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexDepthCam position by VertexTrackXYZ");
    VertexSE3 *cam = dynamic_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = dynamic_cast<VertexPointXYZ*>(_vertices[1]);

    // VertexCameraCache* vcache = (VertexCameraCache* ) cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }
    //ParameterCamera* params=vcache->params;
    const Eigen::Matrix<double, 3, 3>& invKcam = params->invKcam();
    Eigen::Vector3d p;
    double w=1./_measurement(2);
    p.head<2>() = _measurement.head<2>()*w;
    p(2) = w;
    p = invKcam * p;
    p = cam->estimate() * (params->offsetMatrix() * p);
    point->setEstimate(p);
  }


#ifdef G2O_HAVE_OPENGL
  EdgeProjectDisparityDrawAction::EdgeProjectDisparityDrawAction(): DrawAction(typeid(EdgeSE3PointXYZDisparity).name()){}

  HyperGraphElementAction* EdgeProjectDisparityDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                HyperGraphElementAction::Parameters* /* params_ */){
    return 0;
    if (typeid(*element).name()!=_typeName)
      return 0;
    EdgeSE3PointXYZDisparity* e =  static_cast<EdgeSE3PointXYZDisparity*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertex(0));
    VertexPointXYZ* toEdge   = static_cast<VertexPointXYZ*>(e->vertex(1));
    glColor3f(0.4f,0.4f,0.2f);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),(float)fromEdge->estimate().translation().z());
    glVertex3f((float)toEdge->estimate().x(),(float)toEdge->estimate().y(),(float)toEdge->estimate().z());
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

}
}
