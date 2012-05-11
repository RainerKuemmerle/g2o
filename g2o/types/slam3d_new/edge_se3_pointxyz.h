#ifndef G2O_EDGE_SE3_POINT_XYZ_NEW_H_
#define G2O_EDGE_SE3_POINT_XYZ_NEW_H_

#include "g2o/core/base_binary_edge.h"

#include "vertex_se3.h"
#include "vertex_pointxyz.h"
#include "parameter_se3_offset.h"

namespace Slam3dNew {
  using namespace g2o;


  /*! \class EdgeSE3TrackXYZ
   * \brief g2o edge from a track to a point node
   */
  // first two args are the measurement type, second two the connection classes
  class EdgeSE3PointXYZ : public BaseBinaryEdge<3, Vector3d, VertexSE3, VertexPointXYZ> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3PointXYZ();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError();
    // jacobian
    virtual void linearizeOplus();
    

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
    ParameterSE3Offset* offsetParam;
    CacheSE3Offset* cache;
    virtual bool resolveCaches();
    

  };

}
#endif
