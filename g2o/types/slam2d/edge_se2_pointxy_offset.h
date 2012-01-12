#ifndef G2O_EDGE_SE2_POINT_XY_OFFSET_H_
#define G2O_EDGE_SE2_POINT_XY_OFFSET_H_

#include "g2o/core/base_binary_edge.h"

#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "parameter_se2_offset.h"

namespace g2o {


  /*! \class EdgeSE2TrackXYZ
   * \brief g2o edge from a track to a point node
   */
  // first two args are the measurement type, second two the connection classes
  class EdgeSE2PointXYOffset : public BaseBinaryEdge<2, Vector2d, VertexSE2, VertexPointXY> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE2PointXYOffset();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError();
    // jacobian
    virtual void linearizeOplus();
    

    virtual void setMeasurement(const Vector2d& m){
      _measurement = m;
    }

    virtual bool setMeasurementData(const double* d){
      Map<const Vector2d> v(d);
      _measurement = v;
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      Map<Vector2d> v(d);
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
    ParameterSE2Offset* offsetParam;
    CacheSE2Offset* cache;
    virtual bool resolveCaches();

  };

}
#endif
