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

#ifndef G2O_EDGE_SE2_POINT_XY_H
#define G2O_EDGE_SE2_POINT_XY_H

#include "g2o/config.h"
#include "vertex_se2.h"
#include "vertex_point_xy.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

  class G2O_TYPES_SLAM2D_API EdgeSE2PointXY : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2PointXY();

      void computeError()
      {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexPointXY* l2 = static_cast<const VertexPointXY*>(_vertices[1]);
        _error = (v1->estimate().inverse() * l2->estimate()) - _measurement;
      }

      virtual bool setMeasurementData(const double* d){
  _measurement[0]=d[0];
  _measurement[1]=d[1];
  return true;
      }

      virtual bool getMeasurementData(double* d) const{
  d[0] = _measurement[0];
  d[1] = _measurement[1];
  return true;
      }
      
      virtual int measurementDimension() const {return 2;}

      virtual bool setMeasurementFromState(){
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexPointXY* l2 = static_cast<const VertexPointXY*>(_vertices[1]);
  _measurement = v1->estimate().inverse() * l2->estimate();
  return true;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);}
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      virtual void linearizeOplus();
#endif
  };

  class G2O_TYPES_SLAM2D_API EdgeSE2PointXYWriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeSE2PointXYWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgeSE2PointXYDrawAction: public DrawAction{
  public:
    EdgeSE2PointXYDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace

#endif
