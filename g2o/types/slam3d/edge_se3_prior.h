#ifndef G2O_EDGE_SE3_PRIOR_H_
#define G2O_EDGE_SE3_PRIOR_H_

#include "vertex_se3_quat.h"
#include "g2o/core/base_unary_edge.h"
#include "parameter_se3_offset.h"
namespace g2o {

  /**
   * prior for an SE3 element
   */
  // first two args are the measurement type, third connection class
  class G2O_TYPES_SLAM3D_API EdgeSE3Prior : public BaseUnaryEdge<6, SE3Quat, VertexSE3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3Prior();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError();
    
    // jacobian
    virtual void linearizeOplus();

    virtual void setMeasurement(const SE3Quat& m){
      _measurement = m;
      _inverseMeasurement = m.inverse();
    }

    virtual bool setMeasurementData(const double* d){
      Map<const Vector7d> v(d);
      _measurement.fromVector(v);
      _inverseMeasurement = _measurement.inverse();
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      Map<Vector7d> v(d);
      v = _measurement.toVector();
      return true;
    }
    
    virtual int measurementDimension() const {return 7;}

    virtual bool setMeasurementFromState() ;

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
             OptimizableGraph::Vertex* /*to*/) { 
      return 1.;
    }

    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
  protected:
    SE3Quat _inverseMeasurement;
    virtual bool resolveCaches();
    ParameterSE3Offset* _offsetParam;
    CacheSE3Offset* _cache;
  };

}
#endif
