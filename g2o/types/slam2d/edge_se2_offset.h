#ifndef G2O_EDGE_SE2_OFFSET_H_
#define G2O_EDGE_SE2_OFFSET_H_

#include "g2o/core/base_binary_edge.h"

#include "vertex_se2.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

  class ParameterSE2Offset;
  class CacheSE2Offset;

  /**
   * \brief Offset edge
   */
  // first two args are the measurement type, second two the connection classes
  class G2O_TYPES_SLAM2D_API EdgeSE2Offset : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2Offset();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      // jacobian
      //virtual void linearizeOplus();

      virtual void setMeasurement(const SE2& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

      virtual bool setMeasurementData(const double* d){
        Map<const Vector3d> v(d);
        _measurement.fromVector(v);
        _inverseMeasurement = _measurement.inverse();
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Map<Vector3d> v(d);
        v = _measurement.toVector();
        return true;
      }

      virtual int measurementDimension() const {return 3;}

      virtual bool setMeasurementFromState() ;

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
          OptimizableGraph::Vertex* /*to*/) { 
        return 1.;
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

    protected:
      SE2 _inverseMeasurement;
      virtual bool resolveCaches();
      ParameterSE2Offset *_offsetFrom, *_offsetTo;
      CacheSE2Offset  *_cacheFrom, *_cacheTo;
  };

} // end namespace
#endif
