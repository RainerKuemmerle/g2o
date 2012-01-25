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

#ifndef G2O_EDGE_SE2_H
#define G2O_EDGE_SE2_H

#include "vertex_se2.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_api.h"

namespace g2o {

  /**
   * \brief 2D edge between two Vertex2
   */
  class G2O_TYPES_SLAM2D_API EdgeSE2 : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2();

      void computeError()
      {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSE2* v2 = static_cast<const VertexSE2*>(_vertices[1]);
        SE2 delta = _inverseMeasurement * (v1->estimate().inverse()*v2->estimate());
        _error = delta.toVector();
      }
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void setMeasurement(const SE2& m){
  _measurement = m;
  _inverseMeasurement = m.inverse();
      }

      virtual bool setMeasurementData(const double* d){
  _measurement=SE2(d[0], d[1], d[2]);
  _inverseMeasurement = _measurement.inverse();
  return true;
      }

      virtual bool getMeasurementData(double* d) const {
  Vector3d v=_measurement.toVector();
  d[0] = v[0];
  d[1] = v[1];
  d[2] = v[2];
  return true;
      }

      virtual int measurementDimension() const {return 3;}

      virtual bool setMeasurementFromState() {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSE2* v2 = static_cast<const VertexSE2*>(_vertices[1]);
  _measurement = v1->estimate().inverse()*v2->estimate();
  _inverseMeasurement = _measurement.inverse();
  return true;
      }

      
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 1.;}
      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
      virtual void linearizeOplus();
#endif
    protected:
      SE2 _inverseMeasurement;
  };

  class G2O_TYPES_SLAM2D_API EdgeSE2WriteGnuplotAction: public WriteGnuplotAction {
  public:
    EdgeSE2WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_API EdgeSE2DrawAction: public DrawAction{
  public:
    EdgeSE2DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

} // end namespace

#endif
