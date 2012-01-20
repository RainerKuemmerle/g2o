#ifndef G2O_EDGE_SE2_SENSOR_CALIB_H
#define G2O_EDGE_SE2_SENSOR_CALIB_H

#include "g2o_types_sclam2d_api.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam2d/vertex_se2.h"

namespace g2o {

  /**
   * \brief scanmatch measurement that also calibrates an offset for the laser
   */
  class G2O_TYPES_SCLAM2D_API EdgeSE2SensorCalib : public BaseMultiEdge<3, SE2>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2SensorCalib();

      void computeError()
      {
        const VertexSE2* v1          = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSE2* v2          = static_cast<const VertexSE2*>(_vertices[1]);
        const VertexSE2* laserOffset = static_cast<const VertexSE2*>(_vertices[2]);
        const SE2& x1 = v1->estimate();
        const SE2& x2 = v2->estimate();
        SE2 delta = _inverseMeasurement * ((x1 * laserOffset->estimate()).inverse() * x2 * laserOffset->estimate());
        _error = delta.toVector();
      }

      void setMeasurement(const SE2& m){
  _measurement = m;
  _inverseMeasurement = m.inverse();
      }

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to)
      {
        if (   from.count(_vertices[2]) == 1 // need the laser offset
            && ((from.count(_vertices[0]) == 1 && to == _vertices[1]) || ((from.count(_vertices[1]) == 1 && to == _vertices[0])))) {
          return 1.0;
        }
        return -1.0;
      }
      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

    protected:
      SE2 _inverseMeasurement;
  };

#ifdef G2O_HAVE_OPENGL
  class EdgeSE2SensorCalibDrawAction: public DrawAction {
  public:
    EdgeSE2SensorCalibDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace

#endif
