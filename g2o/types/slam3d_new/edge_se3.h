#ifndef G2O_EDGE_SE3_H_
#define G2O_EDGE_SE3_H_

#include "g2o/core/base_binary_edge.h"

#include "vertex_se3.h"

namespace Slam3dNew {
  using namespace g2o;

  /**
   * \brief Offset edge
   */
  // first two args are the measurement type, second two the connection classes
  class EdgeSE3 : public BaseBinaryEdge<6, Eigen::Isometry3d, VertexSE3, VertexSE3> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      virtual void setMeasurement(const Eigen::Isometry3d& m){
        _measurement = m;
        _inverseMeasurement = m.inverse();
      }

      virtual bool setMeasurementData(const double* d){
        Map<const Vector7d> v(d);
        _measurement=fromVectorQT(v);
        _inverseMeasurement = _measurement.inverse();
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Map<Vector7d> v(d);
        v = toVectorQT(_measurement);
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
      Eigen::Isometry3d _inverseMeasurement;
  };

  class EdgeSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class EdgeSE3DrawAction: public DrawAction{
  public:
    EdgeSE3DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace
#endif
