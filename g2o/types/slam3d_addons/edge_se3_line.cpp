#include "edge_se3_line.h"

namespace Slam3dAddons {
  using namespace g2o;

  EdgeSE3Line3D::EdgeSE3Line3D(){
  }

  bool EdgeSE3Line3D::read(std::istream& is) {
    for (int i=0; i<6; i++)
      is >> _measurement(i);
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
	is >> information()(i,j); 
	information()(i,j) = information()(j,i); 
      }
    return is.good();
  }

  bool EdgeSE3Line3D::write(std::ostream& os) const {
    for (int i=0; i<6; i++)
      os << _measurement(i) << " ";
    for (int i=0; i<6; i++)
      for (int j=i; j<6; j++){
	os << information()(i,j) << " ";
      }
    return os.good();
  }
  
  void EdgeSE3Line3D::computeError() {
    const VertexSE3* pose=static_cast<const VertexSE3*>(vertices()[0]);
    const VertexLine3D* landmark=static_cast<const VertexLine3D*>(vertices()[1]);
    Line3D projected(pose->estimate() * _measurement);
    projected.normalize();
    _error = landmark->estimate() - projected;
  }

  // void EdgeSE3Line3D::linearizeOplus() {
  //   return true;
  // }
  
  // virtual bool EdgeSE3Line3D::setMeasurementFromState() {
  //   return true;
  // }

  // virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to){
  // }

}
