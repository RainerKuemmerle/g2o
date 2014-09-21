#ifndef G2O_EDGE_SE2_TWOPOINTS_XY_H
#define G2O_EDGE_SE2_TWOPOINTS_XY_H

#include "g2o/config.h"
#include "g2o_types_slam2d_api.h"
#include "g2o/core/base_multi_edge.h"
#include "vertex_se2.h"
#include "vertex_point_xy.h"

namespace g2o{

  class G2O_TYPES_SLAM2D_API EdgeSE2TwoPointsXY : public BaseMultiEdge<4, Vector4D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2TwoPointsXY();

      virtual void computeError();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual bool setMeasurementFromState();

      virtual void initialEstimate(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*);
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*);
  };
}
#endif	// G2O_EDGE_SE2_TWOPOINTS_XY_H
