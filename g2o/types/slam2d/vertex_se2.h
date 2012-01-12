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

#ifndef G2O_VERTEX_SE2_H
#define G2O_VERTEX_SE2_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2.h"

namespace g2o {

  /**
   * \brief 2D pose Vertex, (x,y,theta)
   */
  class VertexSE2 : public BaseVertex<3, SE2>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VertexSE2();

      virtual void setToOriginImpl() {
        _estimate = SE2();
      }

      virtual void oplusImpl(const double* update)
      {
        Vector2d t=_estimate.translation();
        t+=Map<const Vector2d>(update);
        double angle=normalize_theta(_estimate.rotation().angle() + update[2]);
        _estimate.setTranslation(t);
        _estimate.setRotation(Rotation2Dd(angle));
      }

      virtual bool setEstimateDataImpl(const double* est){
        _estimate=SE2(est[0], est[1], est[2]);
        return true;
      }

      virtual bool getEstimateData(double* est) const {
        Map<Vector3d> v(est);
        v = _estimate.toVector();
        return true;
      }
      
      virtual int estimateDimension() const { return 3; }

      virtual bool setMinimalEstimateDataImpl(const double* est){
        return setEstimateData(est);
      }

      virtual bool getMinimalEstimateData(double* est) const {
  return getEstimateData(est);
      }
      
      virtual int minimalEstimateDimension() const { return 3; }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

  };

  class VertexSE2WriteGnuplotAction: public WriteGnuplotAction {
  public:
    VertexSE2WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  class VertexSE2DrawAction: public DrawAction{
  public:
    VertexSE2DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  };
#endif

} // end namespace

#endif
