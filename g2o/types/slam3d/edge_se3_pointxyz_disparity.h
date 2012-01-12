#ifndef G2O_EDGE_SE3_POINTXYZ_DISPARITY_H_
#define G2O_EDGE_SE3_POINTXYZ_DISPARITY_H_

#include "vertex_se3_quat.h"
#include "vertex_pointxyz.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/base_binary_edge.h"
#include "parameter_camera.h"

#define EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN
namespace g2o {


  /**
   * edge from a track to a depth camera node using a disparity measurement
   *
   * the disparity measurement is normalized: disparity / (focal_x * baseline)
   */
  // first two args are the measurement type, second two the connection classes
  class EdgeSE3PointXYZDisparity : public BaseBinaryEdge<3, Vector3d, VertexSE3, VertexPointXYZ> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3PointXYZDisparity();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError();

#ifdef EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN
    virtual void linearizeOplus();
#endif

    virtual void setMeasurement(const Vector3d& m){
      _measurement = m;
    }

    virtual bool setMeasurementData(const double* d){
      Map<const Vector3d> v(d);
      _measurement = v;
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      Map<Vector3d> v(d);
      v=_measurement;
      return true;
    }
    
    virtual int measurementDimension() const {return 3;}

    virtual bool setMeasurementFromState() ;
    
    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, 
             OptimizableGraph::Vertex* to) { 
      (void) to; 
      return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);
    }

    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

  private:
    Eigen::Matrix<double,3,9> J; // jacobian before projection
    virtual bool resolveCaches();
    ParameterCamera* params;
    CacheCamera* cache;
  };


#ifdef G2O_HAVE_OPENGL
  class EdgeProjectDisparityDrawAction: public DrawAction{
  public:
    EdgeProjectDisparityDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

}
#endif

/* #if 0 */

/*   class EdgeProjectDisparitySensorCalib : public BaseMultiEdge<3, Vector3d> */
/*   { */
/*     public: */
/*       EIGEN_MAKE_ALIGNED_OPERATOR_NEW */
/*         EdgeProjectDisparitySensorCalib(); */

/*       virtual void computeError(); */
/*       virtual bool read(std::istream& is); */
/*       virtual bool write(std::ostream& os) const; */
      
/*   }; */

/*   EdgeProjectDisparitySensorCalib::EdgeProjectDisparitySensorCalib() { */
/*     information().setIdentity(); */
/*     information()(2,2)=1000.; */
/*   } */
  
/*   void EdgeProjectDisparitySensorCalib::computeError() */
/*   { */
/*     const VertexSE3* v1          = static_cast<const VertexSE3*>(_vertices[0]); */
/*     const VertexTrackXYZ* v2          = static_cast<const VertexTrackXYZ*>(_vertices[1]); */
/*     const VertexSE3* v3           = static_cast<const VertexSE3*>(_vertices[2]); */
/*     const DepthCam& robotPose   = v1->estimate();// * laserOffset->estimate(); */
/*     const Vector3d& point = v2->estimate(); */
/*     const SE3Quat& offset = v3->estimate(); */
/*     Vector3d p=robotPose.Kcam*(offset.inverse()*(robotPose.inverse()*point)); */
/*     double iw=1./p(2); */
/*     p.head<2>()*=iw; */
/*     p(2)=iw; */
/*     _error = p - _measurement; */
/*   } */


/*   bool EdgeProjectDisparitySensorCalib::read(std::istream& is) { */
/*     // measured keypoint */
/*     for (int i=0; i<3; i++) is >> measurement()[i]; */
/*     // don't need this if we don't use it in error calculation (???) */
/*     // information matrix is the identity for features, could be changed to allow arbitrary covariances */
/*     if (is.bad()) */
/*       return false; */
/*     for ( int i=0; i<information().rows() && is.good(); i++) */
/*       for (int j=i; j<information().cols() && is.good(); j++){ */
/*   is >> information()(i,j); */
/*   if (i!=j) */
/*     information()(j,i)=information()(i,j); */
/*       } */
/*     if (is.bad()) { */
/*       //  we overwrite the information matrix */
/*       information().setIdentity(); */
/*       information()(2,2)=1000.; */
/*     } */
/*     return true; */
/*   } */

/*   bool EdgeProjectDisparitySensorCalib::write(std::ostream& os) const { */
/*     for (int i=0; i<3; i++) os  << measurement()[i] << " "; */
/*     for (int i=0; i<information().rows(); i++) */
/*       for (int j=i; j<information().cols(); j++) { */
/*         os <<  information()(i,j) << " "; */
/*       } */
/*     return os.good(); */
/*   } */
/* #endif */
