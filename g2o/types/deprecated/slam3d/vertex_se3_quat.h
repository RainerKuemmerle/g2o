// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_DEPRECATED_VERTEX_SE3_QUAT_
#define G2O_DEPRECATED_VERTEX_SE3_QUAT_

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o_deprecated_types_slam3d_api.h"

namespace g2o {
namespace deprecated {


/**
 * \brief 3D pose Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
class G2O_DEPRECATED_TYPES_SLAM3D_API VertexSE3 : public BaseVertex<6, SE3Quat>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexSE3();

    virtual void setToOriginImpl() {
      _estimate = SE3Quat();
    }

    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;


    virtual bool setEstimateDataImpl(const double* est){
      Eigen::Map<const Vector7> v(est);
      _estimate.fromVector(v);
      return true;
    }

    virtual bool getEstimateData(double* est) const{
      Eigen::Map<Vector7> v(est);
      v=_estimate.toVector();
      return true;
    }

    virtual int estimateDimension() const {
      return 7;
    }

    virtual bool setMinimalEstimateDataImpl(const double* est){
      Eigen::Map<const Vector6> v(est);
      _estimate.fromMinimalVector(v);
      return true;
    }

    virtual bool getMinimalEstimateData(double* est) const{
      Eigen::Map<Vector6> v(est);
      v = _estimate.toMinimalVector();
      return true;
    }

    virtual int minimalEstimateDimension() const {
      return 6;
    }

    virtual void oplusImpl(const double* update)
    {
      Eigen::Map<const Vector6> v(update);
      SE3Quat increment(v);
      _estimate *= increment;
    }
};

  class G2O_DEPRECATED_TYPES_SLAM3D_API VertexSE3WriteGnuplotAction: public WriteGnuplotAction {
  public:
    VertexSE3WriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_DEPRECATED_TYPES_SLAM3D_API VertexSE3DrawAction: public DrawAction{
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
} // end namespace

#endif
