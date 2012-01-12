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

#ifndef G2O_VERTEX_POINT_XY_H
#define G2O_VERTEX_POINT_XY_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o {

  class VertexPointXY : public BaseVertex<2, Eigen::Vector2d>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexPointXY();

      virtual void setToOriginImpl() {
        _estimate.setZero();
      }

      virtual bool setEstimateDataImpl(const double* est){
  _estimate[0] = est[0];
  _estimate[1] = est[1];
  return true;
      }

      virtual bool getEstimateData(double* est) const{
  est[0] = _estimate[0];
  est[0] = _estimate[1];
  return true;
      }
      
      virtual int estimateDimension() const { 
  return 2;
      }

      virtual bool setMinimalEstimateDataImpl(const double* est){
  return setEstimateData(est);
      }

      virtual bool getMinimalEstimateData(double* est) const{
  return getEstimateData(est);
      }
      
      virtual int minimalEstimateDimension() const { 
  return 2;
      }

      virtual void oplusImpl(const double* update)
      {
        _estimate[0] += update[0];
        _estimate[1] += update[1];
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

  };

  class VertexPointXYWriteGnuplotAction: public WriteGnuplotAction {
  public:
    VertexPointXYWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class VertexPointXYDrawAction: public DrawAction{
  public:
    VertexPointXYDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

}

#endif
