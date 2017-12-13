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

#ifndef G2O_DEPRECATED_VERTEX_TRACKXYZ_H_
#define G2O_DEPRECATED_VERTEX_TRACKXYZ_H_

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include "g2o_deprecated_types_slam3d_api.h"

namespace g2o {
namespace deprecated {

  /**
   * Vertex for a tracked point in space
   */
  class G2O_DEPRECATED_TYPES_SLAM3D_API VertexPointXYZ : public BaseVertex<3, Vector3>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
          VertexPointXYZ() {}
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        virtual void setToOriginImpl() { _estimate.fill(0.); }

        virtual void oplusImpl(const double* update_) {
          Eigen::Map<const Vector3> update(update_);
          _estimate += update;
        }

        virtual bool setEstimateDataImpl(const double* est){
          Eigen::Map<const Vector3> _est(est);
          _estimate = _est;
          return true;
        }

        virtual bool getEstimateData(double* est) const{
          Eigen::Map<Vector3> _est(est);
          _est = _estimate;
          return true;
        }

        virtual int estimateDimension() const {
          return 3;
        }

        virtual bool setMinimalEstimateDataImpl(const double* est){
          _estimate = Eigen::Map<const Vector3>(est);
          return true;
        }

        virtual bool getMinimalEstimateData(double* est) const{
          Eigen::Map<Vector3> v(est);
          v = _estimate;
          return true;
        }

        virtual int minimalEstimateDimension() const {
          return 3;
        }

    };

  class G2O_DEPRECATED_TYPES_SLAM3D_API VertexPointXYZWriteGnuplotAction: public WriteGnuplotAction
  {
    public:
      VertexPointXYZWriteGnuplotAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_DEPRECATED_TYPES_SLAM3D_API VertexPointXYZDrawAction: public DrawAction{
  public:
    VertexPointXYZDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);

    
  protected:
    FloatProperty *_pointSize;
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
  };
#endif

}
}
#endif
