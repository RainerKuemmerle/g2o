#ifndef G2O_SE3_LOTSOF_XYZ
#define G2O_SE3_LOTSOF_XYZ

#include "g2o/config.h"
#include "g2o_types_slam3d_api.h"
#include "g2o/core/base_multi_edge.h"
#include "vertex_se3.h"
#include "vertex_pointxyz.h"

namespace g2o{

  class G2O_TYPES_SLAM3D_API EdgeSE3LotsOfXYZ : public BaseMultiEdge<-1, VectorXD>{

    protected:
      unsigned int _observedPoints;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3LotsOfXYZ();

      void setDimension(int dimension_){
        _dimension = dimension_;
        _information.resize(dimension_, dimension_);
        _error.resize(dimension_, 1);
        _measurement.resize(dimension_, 1);
      }

      void setSize(int vertices){
        resize(vertices);
        _observedPoints = vertices-1;
        setDimension(_observedPoints*3);
      }


      virtual void computeError();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual bool setMeasurementFromState();

      virtual void initialEstimate(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*);
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*);

      virtual void linearizeOplus();

  };

}



#endif // G2O_SE3_LOTSOF_XYZ
