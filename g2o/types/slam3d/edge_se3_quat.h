// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_EDGE_SE3_QUAT_
#define G2O_EDGE_SE3_QUAT_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/hyper_graph_action.h"
#include "se3quat.h"

#define EDGE_SE3_QUAT_ANALYTIC_JACOBIAN

#include "vertex_se3_quat.h"

namespace g2o {

  using namespace Eigen;


/**
 * \brief 3D edge between two VertexSE3
 */
 class EdgeSE3 : public BaseBinaryEdge<6, SE3Quat, VertexSE3, VertexSE3>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3();
    void computeError()
    {
      const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
      const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
      SE3Quat delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
      _error.head<3>() = delta.translation();
      // The analytic Jacobians assume the error in this special form (w beeing positive)
      if (delta.rotation().w() < 0.)
        _error.tail<3>() =  - delta.rotation().vec();
      else
        _error.tail<3>() =  delta.rotation().vec();
    }

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
    
    virtual int measurementDimension() const {return 7;}

    virtual bool setMeasurementFromState() ;

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

#ifdef EDGE_SE3_QUAT_ANALYTIC_JACOBIAN
    virtual void linearizeOplus();
#endif
  protected:
    SE3Quat _inverseMeasurement;
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
