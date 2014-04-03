#include "edge_se3_lotsofxyz.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o{


  EdgeSE3LotsOfXYZ::EdgeSE3LotsOfXYZ(){
    resize(0);
  }

  
  bool EdgeSE3LotsOfXYZ::setMeasurementFromState(){
    VertexSE3 * pose = static_cast<VertexSE3 *> (_vertices[0]);
    
    Eigen::Transform<double, 3, 1> poseinv = pose->estimate().inverse();
    
    for(unsigned int i=0; i<_observedPoints; i++){
      VertexPointXYZ * xyz = static_cast<VertexPointXYZ *> (_vertices[1+i]);
      //      const Eigen::Vector3d &pt = xyz->estimate();
      Eigen::Vector3d m = poseinv * xyz->estimate();
      
      unsigned int index = 3*i;
      _measurement[index] = m[0];
      _measurement[index+1] = m[1];
      _measurement[index+2] = m[2];
    }
    return true;
  }
  
  
  
  void EdgeSE3LotsOfXYZ::computeError(){
    VertexSE3 * pose = static_cast<VertexSE3 *> (_vertices[0]);
    
    for(unsigned int i=0; i<_observedPoints; i++){
      VertexPointXYZ * xyz = static_cast<VertexPointXYZ *> (_vertices[1+i]);
      Eigen::Vector3d m = pose->estimate().inverse() * xyz->estimate();
      
      unsigned int index = 3*i;
      _error[index] = m[0] - _measurement[index];
      _error[index+1] = m[1] - _measurement[index+1];
      _error[index+2] = m[2] - _measurement[index+2];
    }
  }
  
  
  
  // void EdgeSE3LotsOfXYZ::linearizeOplus(){
    
  //   //    BaseMultiEdge::linearizeOplus();

  //   g2o::VertexSE3 * pose = (g2o::VertexSE3 *) (_vertices[0]);
    
  //   // initialize Ji matrix
  //   Eigen::MatrixXd Ji;
  //   unsigned int rows = 3*(_vertices.size()-1);
  //   Ji.resize(rows, 6);
  //   Ji.fill(0);
    
  //   //    Eigen::Matrix3d poseRot = pose->estimate().inverse().rotation();
    
  //   double p[7];
  //   pose->getEstimateData(p);
  //   double ex=p[3];
  //   double ey=p[4];
  //   double ez=p[5];
  //   double w =p[6];
  //   Eigen::Matrix3d A;
  //   A <<
  //     (2*(w*w + ex*ex) - 1), (2*(ex*ey + w*ez)), (2*(ex*ez - w*ey)),
  //     (2*(ex*ey - w*ez)), (2*(w*w + ey*ey) - 1), (2*(ey*ez + w*ex)),
  //     (2*(ex*ez + w*ey)), (2*(ey*ez - w*ex)), (2*(w*w + ez*ez) - 1);
    
  //   // std::cout << "poseRot:" << std::endl << poseRot << std::endl << "A:" << std::endl << A << std::endl;
    
  //   Eigen::Matrix3d dAx;
  //   Eigen::Matrix3d dAy;
  //   Eigen::Matrix3d dAz;
  //   dAx <<
  //     4*ex, 2*ey, 2*ez,
  //     2*ey, 0, 2*w,
  //     2*ez, -2*w, 0;
    
  //   dAy <<
  //     0, 2*ex, -2*w,
  //     2*ex, 4*ey, 2*ez,
  //     2*w, 2*ez, 0;
    
  //   dAz <<
  //     0, 2*w, 2*ey,
  //     -2*w, 0, 2*ey,
  //     2*ex, 2*ey, 4*ez;
    
  //   // Eigen::MatrixXd Jg;
  //   // Jg.resize(rows, 6);
  //   // Jg.fill(0);
    
    
  //   for(unsigned int i=1; i<_vertices.size(); i++){
  //     unsigned int index=3*(i-1); // row index in the jacobian
      
  //     g2o::VertexPointXYZ * point = (g2o::VertexPointXYZ *) (_vertices[i]);
  //     double l[3];
  //     point->getEstimateData(l);
      
  //     // Ji.block<3,3>(index,0) = -A;
  //     Ji.block<3,3>(index,0) = -Eigen::Matrix3d::Identity(); // this is because the oPlus method considers the relative displacement
      
  //     Ji(index, 3) = dAx(0,0) * (l[0] - p[0]) + dAx(0,1) * (l[1] - p[1]) + dAx(0,2) * (l[2] - p[2]);
  //     Ji(index, 4) = dAy(0,0) * (l[0] - p[0]) + dAy(0,1) * (l[1] - p[1]) + dAy(0,2) * (l[2] - p[2]);
  //     Ji(index, 5) = dAz(0,0) * (l[0] - p[0]) + dAz(0,1) * (l[1] - p[1]) + dAz(0,2) * (l[2] - p[2]);
      
  //     Ji(index+1, 3) = dAx(1,0) * (l[0] - p[0]) + dAx(1,1) * (l[1] - p[1]) + dAx(1,2) * (l[2] - p[2]);
  //     Ji(index+1, 4) = dAy(1,0) * (l[0] - p[0]) + dAy(1,1) * (l[1] - p[1]) + dAy(1,2) * (l[2] - p[2]);
  //     Ji(index+1, 5) = dAz(1,0) * (l[0] - p[0]) + dAz(1,1) * (l[1] - p[1]) + dAz(1,2) * (l[2] - p[2]);
      
  //     Ji(index+2, 3) = dAx(2,0) * (l[0] - p[0]) + dAx(2,1) * (l[1] - p[1]) + dAx(2,2) * (l[2] - p[2]);
  //     Ji(index+2, 4) = dAy(2,0) * (l[0] - p[0]) + dAy(2,1) * (l[1] - p[1]) + dAy(2,2) * (l[2] - p[2]);
  //     Ji(index+2, 5) = dAz(2,0) * (l[0] - p[0]) + dAz(2,1) * (l[1] - p[1]) + dAz(2,2) * (l[2] - p[2]);
      
  //     // Eigen::Vector3d Zcam = pose->estimate().inverse() * point->estimate();
      
  //     // Jg.block<3,3>(index,0) = -Eigen::Matrix3d::Identity();
      
  //     // Jg(index, 3) = -0.0;
  //     // Jg(index, 4) = -2*Zcam(2);
  //     // Jg(index, 5) = 2*Zcam(1);
      
  //     // Jg(index+1, 3) = 2*Zcam(2);
  //     // Jg(index+1, 4) = -0.0;
  //     // Jg(index+1, 5) = -2*Zcam(0);
      
  //     // Jg(index+2, 3) = -2*Zcam(1);
  //     // Jg(index+2, 4) = 2*Zcam(0);
  //     // Jg(index+2, 5) = -0.0;
      
  //     Eigen::MatrixXd Jj;
  //     Jj.resize(rows, 3);
  //     Jj.fill(0);
  //     Jj.block<3,3>(index,0) = A;
      
  //     _jacobianOplus[i] = Jj;
  //   }
    
  //   // std::cout << "numerically computed jacobian:" << std::endl << _jacobianOplus[0] << std::endl << "'my' jacobian:" << std::endl <<  Ji << std::endl << "num - mine" << std::endl << _jacobianOplus[0] - Ji << std::endl << std::endl;
  //   // std::cout << "Giorgio's style jacobian:" << std::endl << Jg << std::endl << "Jg - Ji:" << std::endl << Jg-Ji << std::endl << "num - Jg:" << std::endl << _jacobianOplus[0] - Jg << std::endl << std::endl;
  //   _jacobianOplus[0] = Ji;
    
  // }
  
  
  void EdgeSE3LotsOfXYZ::linearizeOplus(){
    
    g2o::VertexSE3 * pose = (g2o::VertexSE3 *) (_vertices[0]);
    
    // initialize Ji matrix
    Eigen::MatrixXd Ji;
    unsigned int rows = 3*(_vertices.size()-1);
    Ji.resize(rows, 6);
    Ji.fill(0);
    
    Eigen::Matrix3d poseRot = pose->estimate().inverse().rotation();
    
    for(unsigned int i=1; i<_vertices.size(); i++){
      g2o::VertexPointXYZ * point = (g2o::VertexPointXYZ *) (_vertices[i]);
      Eigen::Vector3d Zcam = pose->estimate().inverse() * point->estimate();
      
      unsigned int index=3*(i-1);
      
      // Ji.block<3,3>(index,0) = -poseRot;
      Ji.block<3,3>(index,0) = -Eigen::Matrix3d::Identity();
      
      Ji(index, 3) = -0.0;
      Ji(index, 4) = -2*Zcam(2);
      Ji(index, 5) = 2*Zcam(1);
      
      Ji(index+1, 3) = 2*Zcam(2);
      Ji(index+1, 4) = -0.0;
      Ji(index+1, 5) = -2*Zcam(0);
      
      Ji(index+2, 3) = -2*Zcam(1);
      Ji(index+2, 4) = 2*Zcam(0);
      Ji(index+2, 5) = -0.0;
      
      Eigen::MatrixXd Jj;
      Jj.resize(rows, 3);
      Jj.fill(0);
      Jj.block<3,3>(index,0) = poseRot;
      
      _jacobianOplus[i] = Jj;
    }
    
    _jacobianOplus[0] = Ji;
    
  }
  
  bool EdgeSE3LotsOfXYZ::read(std::istream& is){
    is >> _observedPoints;

    setSize(_observedPoints + 1);
    
    // read the measurements
    for(unsigned int i=0; i<_observedPoints; i++){
      unsigned int index = 3*i;
      is >> _measurement[index] >> _measurement[index+1] >> _measurement[index+2];
    }
    
    // read the information matrix
    for(unsigned int i=0; i<_observedPoints*3; i++){
      // fill the "upper triangle" part of the matrix
      for(unsigned int j=i; j<_observedPoints*3; j++){
	is >> information()(i,j);
      }
      
      // fill the lower triangle part
      for(unsigned int j=0; j<i; j++){
	information()(i,j) = information()(j,i);
      }
      
    }
    return true;
  }
  
  
  
  bool EdgeSE3LotsOfXYZ::write(std::ostream& os) const{
    // write number of observed points
    os << "|| " << _observedPoints;
    
    // write measurements
    for(unsigned int i=0; i<_observedPoints; i++){
      unsigned int index = 3*i;
      os << " " << _measurement[index] << " " << _measurement[index+1] << " " << _measurement[index+2];
    }
    
    // write information matrix
    for(unsigned int i=0; i<_observedPoints*3; i++){
      for(unsigned int j=i; j<_observedPoints*3; j++){
	os << " " << information()(i,j);
      }
    }
    return os.good();
  }
  
  
  
  void EdgeSE3LotsOfXYZ::initialEstimate(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
    (void) toEstimate;
    
    assert(initialEstimatePossible(fixed, toEstimate) && "Bad vertices specified");
    
    VertexSE3 * pose = static_cast<VertexSE3 *>(_vertices[0]);
    
    bool estimate_this[_observedPoints];
    for(unsigned int i=0; i<_observedPoints; i++){
      estimate_this[i] = true;
    }
    
    for(std::set<HyperGraph::Vertex*>::iterator it=fixed.begin(); it!=fixed.end(); it++){
      for(unsigned int i=1; i<_vertices.size(); i++){
	VertexPointXYZ * vert = static_cast<VertexPointXYZ *>(_vertices[i]);
	if(vert->id() == (*it)->id()) estimate_this[i-1] = false;
      } 
    }
    
    for(unsigned int i=1; i<_vertices.size(); i++){
      if(estimate_this[i-1]){
	unsigned int index = 3*(i-1);
	Eigen::Vector3d submeas(_measurement[index], _measurement[index+1], _measurement[index+2]);
	VertexPointXYZ * vert = static_cast<VertexPointXYZ *>(_vertices[i]);
	vert->setEstimate(pose->estimate() * submeas);
      }
    }
  }
  
  
  
  double EdgeSE3LotsOfXYZ::initialEstimatePossible(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
    (void) toEstimate;
    
    for(std::set<HyperGraph::Vertex *>::iterator it=fixed.begin(); it!=fixed.end(); it++){
      if(_vertices[0]->id() == (*it)->id()){
	return 1.0;
      }
    }
    
    return -1.0;
  }
}
