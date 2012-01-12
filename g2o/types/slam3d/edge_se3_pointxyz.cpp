#include "edge_se3_pointxyz.h"
#include "parameter_se3_offset.h"
#include <iostream>

namespace g2o {
  using namespace std;


  // point to camera projection, monocular
  EdgeSE3PointXYZ::EdgeSE3PointXYZ() : BaseBinaryEdge<3, Vector3d, VertexSE3, VertexPointXYZ>() {
    information().setIdentity();
    J.fill(0);
    J.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
    cache = 0;
    offsetParam = 0;
    resizeParameters(1);
    installParameter(offsetParam, 0);
  }

  bool EdgeSE3PointXYZ::resolveCaches(){
    ParameterVector pv(1);
    pv[0]=offsetParam;
    resolveCache(cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE3_OFFSET",pv);
    return cache != 0;
  }


  bool EdgeSE3PointXYZ::read(std::istream& is) {
    int pId;
    is >> pId;
    setParameterId(0, pId);
    // measured keypoint
    Vector3d meas;
    for (int i=0; i<3; i++) is >> meas[i];
    setMeasurement(meas);
    // information matrix is the identity for features, could be changed to allow arbitrary covariances    
    if (is.bad()) {
      return false;
    }
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
  is >> information()(i,j);
  if (i!=j)
    information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix
      information().setIdentity();
    } 
    return true;
  }

  bool EdgeSE3PointXYZ::write(std::ostream& os) const {
    os << offsetParam->id() << " ";
    for (int i=0; i<3; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3PointXYZ::computeError() {
    // from cam to point (track)
    //VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = static_cast<VertexPointXYZ*>(_vertices[1]);

    Eigen::Vector3d perr = cache->w2nMatrix() * point->estimate();

    // error, which is backwards from the normal observed - calculated
    // _measurement is the measured projection
    _error = perr - _measurement;
    //    std::cout << _error << std::endl << std::endl;
  }

  void EdgeSE3PointXYZ::linearizeOplus() {
    //VertexSE3 *cam = static_cast<VertexSE3 *>(_vertices[0]);
    VertexPointXYZ *vp = static_cast<VertexPointXYZ *>(_vertices[1]);

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

    Eigen::Matrix<double,3,9> Jhom = offsetParam->inverseOffsetMatrix().rotation() * J;

    _jacobianOplusXi = Jhom.block<3,6>(0,0);
    _jacobianOplusXj = Jhom.block<3,3>(0,6);
  }


  bool EdgeSE3PointXYZ::setMeasurementFromState(){
    //VertexSE3 *cam = static_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = static_cast<VertexPointXYZ*>(_vertices[1]);

    // calculate the projection
    const Vector3d &pt = point->estimate();
    // SE3OffsetCache* vcache = (SE3OffsetCache*) cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }

    Eigen::Vector3d perr = cache->w2nMatrix() * pt;
    _measurement = perr;
    return true;
  }


  void EdgeSE3PointXYZ::initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to_*/)
  {
    (void) from;
    assert(from.size() == 1 && from.count(_vertices[0]) == 1 && "Can not initialize VertexDepthCam position by VertexTrackXYZ");

    VertexSE3 *cam = dynamic_cast<VertexSE3*>(_vertices[0]);
    VertexPointXYZ *point = dynamic_cast<VertexPointXYZ*>(_vertices[1]);
    // SE3OffsetCache* vcache = (SE3OffsetCache* ) cam->getCache(_cacheIds[0]);
    // if (! vcache){
    //   cerr << "fatal error in retrieving cache" << endl;
    // }
    // SE3OffsetParameters* params=vcache->params;
    Eigen::Vector3d p=_measurement;
    point->setEstimate(cam->estimate() * (offsetParam->offsetMatrix() * p));
  }

}
