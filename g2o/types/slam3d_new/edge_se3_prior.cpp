#include "edge_se3_prior.h"
#include "isometry3d_gradients.h"
#include <iostream>

namespace Slam3dNew {
  using namespace std;
  using namespace g2o;


  // point to camera projection, monocular
  EdgeSE3Prior::EdgeSE3Prior() : BaseUnaryEdge<6, Eigen::Isometry3d, VertexSE3>() {
    setMeasurement(Eigen::Isometry3d::Identity());
    information().setIdentity();
    _cache = 0;
    _offsetParam = 0;
    resizeParameters(1);
    installParameter(_offsetParam, 0);
  }


  bool EdgeSE3Prior::resolveCaches(){
    assert(_offsetParam);
    ParameterVector pv(1);
    pv[0]=_offsetParam;
    resolveCache(_cache, (OptimizableGraph::Vertex*)_vertices[0],"CACHE_SE3_OFFSET_NEW",pv);
    return _cache != 0;
  }



  bool EdgeSE3Prior::read(std::istream& is) {
    int pid;
    is >> pid;
    if (!setParameterId(0, pid))
      return false;
    // measured keypoint
    Vector7d meas;
    for (int i=0; i<7; i++) is >> meas[i];
    setMeasurement(fromVectorQT(meas));
    // don't need this if we don't use it in error calculation (???)
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

  bool EdgeSE3Prior::write(std::ostream& os) const {
    os << _offsetParam->id() <<  " ";
    Vector7d meas = toVectorQT(_measurement);
    for (int i=0; i<7; i++) os  << meas[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }


  void EdgeSE3Prior::computeError() {
    Eigen::Isometry3d delta=_inverseMeasurement * _cache->n2w();
    _error = toVectorMQT(delta);
  }

  void EdgeSE3Prior::linearizeOplus(){
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    Eigen::Isometry3d E;
    Eigen::Isometry3d Z, X, P;
    X=from->estimate();
    P=_cache->offsetParam()->offset();
    Z=_measurement;
    computeEdgeSE3PriorGradient(E, _jacobianOplusXi, Z, X, P);
  }


  bool EdgeSE3Prior::setMeasurementFromState(){
    setMeasurement(_cache->n2w());
    return true;
  }


  void EdgeSE3Prior::initialEstimate(const OptimizableGraph::VertexSet& /*from_*/, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3 *v = static_cast<VertexSE3*>(_vertices[0]);

    Eigen::Isometry3d newEstimate = _offsetParam->offset().inverse() * measurement();
    if (_information.block<3,3>(0,0).squaredNorm()!=0){ // do not set translation 
      newEstimate.translation()=v->estimate().translation();
    }
    if (_information.block<3,3>(3,3).squaredNorm()==0){ // do not set rotation
      newEstimate.matrix().block<3,3>(0,0) = v->estimate().rotation();
    }
    v->setEstimate(newEstimate);
  }

}
