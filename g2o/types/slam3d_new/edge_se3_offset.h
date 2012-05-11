#ifndef G2O_EDGE_SE3_OFFSET_H_
#define G2O_EDGE_SE3_OFFSET_H_

#include "g2o/core/base_binary_edge.h"

#include "vertex_se3.h"
#include "edge_se3.h"

namespace Slam3dNew {
  using namespace g2o;
  class ParameterSE3Offset;
  class CacheSE3Offset;

  /**
   * \brief Offset edge
   */
  // first two args are the measurement type, second two the connection classes
  class EdgeSE3Offset : public EdgeSE3 {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3Offset();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();


      void linearizeOplus();

      virtual bool setMeasurementFromState() ;

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/, 
          OptimizableGraph::Vertex* /*to*/) { 
        return 1.;
      }

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

    protected:
      virtual bool resolveCaches();
      ParameterSE3Offset *_offsetFrom, *_offsetTo;
      CacheSE3Offset  *_cacheFrom, *_cacheTo;
  };

} // end namespace
#endif
