#ifndef G2O_EDGE_SE3_POINTXYZ_DISPARITY_H_
#define G2O_EDGE_SE3_POINTXYZ_DISPARITY_H_

#include "vertex_se3.h"
#include "vertex_pointxyz.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/base_binary_edge.h"
#include "parameter_camera.h"

#define EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN
namespace Slam3dNew {
  using namespace g2o;

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
