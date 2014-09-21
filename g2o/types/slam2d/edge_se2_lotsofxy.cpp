#include "edge_se2_lotsofxy.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#endif

namespace g2o{

  EdgeSE2LotsOfXY::EdgeSE2LotsOfXY() : BaseMultiEdge<-1,VectorXD>(){
    resize(0);
  }

  void EdgeSE2LotsOfXY::computeError(){
    VertexSE2 * pose = static_cast<VertexSE2 *> (_vertices[0]);

    for(unsigned int i=0; i<_observedPoints; i++){
      VertexPointXY * xy = static_cast<VertexPointXY *> (_vertices[1+i]);
      Vector2D m = pose->estimate().inverse() * xy->estimate();

      unsigned int index = 2*i;
      _error[index] = m[0] - _measurement[index];
      _error[index+1] = m[1] - _measurement[index+1];
    }
  }

  bool EdgeSE2LotsOfXY::read(std::istream& is){
    is >> _observedPoints;
    setSize(_observedPoints + 1);

    // read the measurements
    for(unsigned int i=0; i<_observedPoints; i++){
      unsigned int index = 2*i;
      is >> _measurement[index] >> _measurement[index+1];
    }

    // read the information matrix
    for(unsigned int i=0; i<_observedPoints*2; i++){
      // fill the "upper triangle" part of the matrix
      for(unsigned int j=i; j<_observedPoints*2; j++){
        is >> information()(i,j);
      }

      // fill the lower triangle part
      for(unsigned int j=0; j<i; j++){
        information()(i,j) = information()(j,i);
      }

    }

    return true;
  }

  bool EdgeSE2LotsOfXY::write(std::ostream& os) const{
    // write number of observed points
    os << "|| " << _observedPoints;

    // write measurements
    for(unsigned int i=0; i<_observedPoints; i++){
      unsigned int index = 2*i;
      os << " " << _measurement[index] << " " << _measurement[index+1];
    }

    // write information matrix
    for(unsigned int i=0; i<_observedPoints*2; i++){
      for(unsigned int j=i; j<_observedPoints*2; j++){
        os << " " << information()(i,j);
      }
    }

    return os.good();
  }


  void EdgeSE2LotsOfXY::linearizeOplus(){
    const VertexSE2* vi     = static_cast<const VertexSE2*>(_vertices[0]);
    const double& x1        = vi->estimate().translation()[0];
    const double& y1        = vi->estimate().translation()[1];
    const double& th1       = vi->estimate().rotation().angle();

    double ct = cos(th1) ;
    double st = sin(th1) ;

    MatrixXD Ji;
    unsigned int rows = 2*(_vertices.size()-1);
    Ji.resize(rows, 3);
    Ji.fill(0);

    Matrix2D poseRot;	// inverse of the rotation matrix associated to the pose
    poseRot <<   ct , st ,
            -st , ct ;

    Matrix2D minusPoseRot = -poseRot;


    for(unsigned int i=1; i<_vertices.size(); i++){
      g2o::VertexPointXY * point = (g2o::VertexPointXY *) (_vertices[i]);

      const double& x2        = point->estimate()[0];
      const double& y2        = point->estimate()[1];

      unsigned int index = 2*(i-1);

      Ji.block<2,2>(index,0) = minusPoseRot;

      Ji(index,2) = ct * (y2-y1) + st * (x1 - x2);
      Ji(index+1,2) = st * (y1-y2) + ct * (x1 - x2);


      MatrixXD Jj;
      Jj.resize(rows, 2);
      Jj.fill(0);
      Jj.block<2,2>(index, 0) = poseRot;

      _jacobianOplus[i] = Jj;
    }
    _jacobianOplus[0] = Ji;
  }


  void EdgeSE2LotsOfXY::initialEstimate(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
    (void) toEstimate;

    assert(initialEstimatePossible(fixed, toEstimate) && "Bad vertices specified");

    VertexSE2 * pose = static_cast<VertexSE2 *>(_vertices[0]);

#ifdef _MSC_VER
	std::vector<bool> estimate_this(_observedPoints, true);
#else
    bool estimate_this[_observedPoints];
    for(unsigned int i=0; i<_observedPoints; i++){
      estimate_this[i] = true;
    }
#endif

    for(std::set<HyperGraph::Vertex*>::iterator it=fixed.begin(); it!=fixed.end(); it++){
      for(unsigned int i=1; i<_vertices.size(); i++){
        VertexPointXY * vert = static_cast<VertexPointXY *>(_vertices[i]);
        if(vert->id() == (*it)->id()) estimate_this[i-1] = false;
      }
    }

    for(unsigned int i=1; i<_vertices.size(); i++){
      if(estimate_this[i-1]){
        unsigned int index = 2*(i-1);
        Vector2D submeas(_measurement[index], _measurement[index+1]);
        VertexPointXY * vert = static_cast<VertexPointXY *>(_vertices[i]);
        vert->setEstimate(pose->estimate() * submeas);
      }
    }
  }


  double EdgeSE2LotsOfXY::initialEstimatePossible(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
    (void) toEstimate;

    for(std::set<HyperGraph::Vertex *>::iterator it=fixed.begin(); it!=fixed.end(); it++){
      if(_vertices[0]->id() == (*it)->id()){
        return 1.0;
      }
    }

    return -1.0;
  }


  bool EdgeSE2LotsOfXY::setMeasurementFromState(){
    VertexSE2 * pose = static_cast<VertexSE2 *> (_vertices[0]);

    for(unsigned int i=0; i<_observedPoints; i++){
      VertexPointXY * xy = static_cast<VertexPointXY *> (_vertices[1+i]);
      Vector2D m = pose->estimate().inverse() * xy->estimate();

      unsigned int index = 2*i;
      _measurement[index] = m[0];
      _measurement[index+1] = m[1];
    }

    return true;
  }

} // end namespace g2o
