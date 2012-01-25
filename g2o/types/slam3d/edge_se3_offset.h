#ifndef G2O_EDGE_SE3_OFFSET_H_
#define G2O_EDGE_SE3_OFFSET_H_

#include "g2o/core/base_binary_edge.h"

#include "vertex_se3_quat.h"

namespace g2o {

  class ParameterSE3Offset;
  class CacheSE3Offset;

  /**
   * \brief Offset edge
   */
  // first two args are the measurement type, second two the connection classes
  class G2O_TYPES_SLAM3D_API EdgeSE3Offset : public BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3Offset();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      // jacobian
      //virtual void linearizeOplus();

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

      void linearizeOplus();

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
      ParameterSE3Offset *_offsetFrom, *_offsetTo;
      CacheSE3Offset  *_cacheFrom, *_cacheTo;
  };

} // end namespace
#endif
