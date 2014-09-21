#ifndef G2O_EDGE_SE2_LOTSOF_XY
#define G2O_EDGE_SE2_LOTSOF_XY

#include "g2o/config.h"
#include "g2o_types_slam2d_api.h"
#include "g2o/core/base_multi_edge.h"
#include "vertex_se2.h"
#include "vertex_point_xy.h"

namespace g2o {

  class G2O_TYPES_SLAM2D_API EdgeSE2LotsOfXY : public BaseMultiEdge<-1,VectorXD>
  {
    protected:
      unsigned int _observedPoints;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2LotsOfXY();

      void setDimension(int dimension_)
      {
        _dimension = dimension_;
        _information.resize(dimension_, dimension_);
        _error.resize(dimension_, 1);
        _measurement.resize(dimension_, 1);
      }

      void setSize(int vertices)
      {
        resize(vertices);
        _observedPoints = vertices-1;
        setDimension(_observedPoints*2);
      }

      virtual void computeError();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual bool setMeasurementFromState();

      virtual void initialEstimate(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*);
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*);

      virtual void linearizeOplus();
  };

} // end namespace g2o

#endif	// G2O_EDGE_SE2_LOTSOF_XY
