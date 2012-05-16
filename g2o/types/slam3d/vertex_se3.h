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

#ifndef G2O_VERTEX_SE3_NEW_
#define G2O_VERTEX_SE3_NEW_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "isometry3d_mappings.h"

namespace g2o {

  using namespace Eigen;

/**
 * \brief 3D pose Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
  class VertexSE3 : public BaseVertex<6, Eigen::Isometry3d>
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexSE3();
      
      virtual void setToOriginImpl() {
        _estimate = Isometry3d::Identity();
      }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;


    virtual bool setEstimateDataImpl(const double* est){
      Map<const Vector7d> v(est);
      _estimate=internal::fromVectorQT(v);
      return true;
    }

    virtual bool getEstimateData(double* est) const{
      Map<Vector7d> v(est);
      v=internal::toVectorQT(_estimate);
      return true;
    }

    virtual int estimateDimension() const {
      return 7;
    }

    virtual bool setMinimalEstimateDataImpl(const double* est){
      Map<const Vector6d> v(est);
      _estimate = internal::fromVectorMQT(v);
      return true;
    }

    virtual bool getMinimalEstimateData(double* est) const{
      Map<Vector6d> v(est);
      v = internal::toVectorMQT(_estimate);
      return true;
    }

    virtual int minimalEstimateDimension() const {
      return 6;
    }

    virtual void oplusImpl(const double* update)
    {
      Map<const Vector6d> v(update);
      Eigen::Isometry3d increment = internal::fromVectorMQT(v);
      _estimate = _estimate * increment;
    }
};

  class VertexSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    VertexSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  class VertexSE3DrawAction: public DrawAction{
  public:
    VertexSE3DrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
    HyperGraphElementAction* _cacheDrawActions;
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    FloatProperty* _triangleX, *_triangleY;
  };
#endif

} // end namespace

#endif
