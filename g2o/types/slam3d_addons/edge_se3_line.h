#ifndef G2O_EDGE_SE3_LINE_H_
#define G2O_EDGE_SE3_LINE_H_

#include "g2o/core/base_binary_edge.h"

#include "vertex_line3d.h"
#include "g2o/types/slam3d/g2o_types_slam3d_api.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace Slam3dAddons {
  using namespace g2o;

  /**
   * \brief Edge between two 3D pose vertices
   *
   * The transformation between the two vertices is given as an Isometry3d.
   * If z denotes the measurement, then the error function is given as follows:
   * z^-1 * (x_i^-1 * x_j)
   */
  class G2O_TYPES_SLAM3D_API EdgeSE3Line3D : public BaseBinaryEdge<6, Line3D, VertexSE3, VertexLine3D> {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE3Line3D();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      void computeError();

      virtual void setMeasurement(const Line3D& m){
        _measurement = Line3D(m);
      }

      virtual bool setMeasurementData(const double* d){
        Map<const Vector6d> v(d);
        setMeasurement(Line3D(v));
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Map<Vector6d> v(d);
        v = _measurement;
        return true;
      }

      //void linearizeOplus();

      virtual int measurementDimension() const {return 6;}
      /*
      virtual bool setMeasurementFromState() ;
      */
      /* virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& /\*from*\/,  */
      /*     OptimizableGraph::Vertex* /\*to*\/) {  */
      /*   return 1.; */
      /* } */

      /* virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to); */

  };

/*   /\** */
/*    * \brief Output the pose-pose constraint to Gnuplot data file */
/*    *\/ */
/*   class G2O_TYPES_SLAM3D_API EdgeSE3WriteGnuplotAction: public WriteGnuplotAction { */
/*   public: */
/*     EdgeSE3WriteGnuplotAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */

/* #ifdef G2O_HAVE_OPENGL */
/*   /\** */
/*    * \brief Visualize a 3D pose-pose constraint */
/*    *\/ */
/*   class G2O_TYPES_SLAM3D_API EdgeSE3DrawAction: public DrawAction{ */
/*   public: */
/*     EdgeSE3DrawAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */
/* #endif */

} // end namespace
#endif
