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

#ifndef G2O_VERTEX_SEGMENT_2D_H
#define G2O_VERTEX_SEGMENT_2D_H

#include "g2o_types_slam2d_addons_api.h"
#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o {

  class G2O_TYPES_SLAM2D_ADDONS_API VertexSegment2D : public BaseVertex<4, Vector4>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexSegment2D();

      Vector2 estimateP1() const { return Eigen::Map<const Vector2>(&(_estimate[0])); }
      Vector2 estimateP2() const { return Eigen::Map<const Vector2>(&(_estimate[2])); }
      void  setEstimateP1(const Vector2& p1){ Eigen::Map<Vector2> v(&_estimate[0]); v=p1; }
      void  setEstimateP2(const Vector2& p2){ Eigen::Map<Vector2> v(&_estimate[2]); v=p2; }

      virtual void setToOriginImpl() {
        _estimate.setZero();
      }

      virtual bool setEstimateDataImpl(const number_t* est){
        Eigen::Map<const Vector4> v(est);
	_estimate = v;
	return true;
      }

      virtual bool getEstimateData(number_t* est) const{	
        Eigen::Map<Vector4> v(est);
	v = _estimate;
	return true;
      }

      virtual int estimateDimension() const {
	return 4;
      }

      virtual bool setMinimalEstimateDataImpl(const number_t* est){
	return setEstimateData(est);
      }

      virtual bool getMinimalEstimateData(number_t* est) const{
	return getEstimateData(est);
      }

      virtual int minimalEstimateDimension() const {
	return 4;
      }

      virtual void oplusImpl(const number_t* update)
      {
        Eigen::Map<const Vector4> upd(update);
        _estimate += upd;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

  };

  class G2O_TYPES_SLAM2D_ADDONS_API VertexSegment2DWriteGnuplotAction: public WriteGnuplotAction {
  public:
    VertexSegment2DWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
            HyperGraphElementAction::Parameters* params_);
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM2D_ADDONS_API VertexSegment2DDrawAction: public DrawAction{
  public:
    VertexSegment2DDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
            HyperGraphElementAction::Parameters* params_);
  protected:
    FloatProperty *_pointSize;
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
  };
#endif

}

#endif
