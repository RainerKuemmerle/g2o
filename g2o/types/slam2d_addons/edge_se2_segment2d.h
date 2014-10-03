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

#ifndef G2O_EDGE_SE2_SEGMENT2D_H
#define G2O_EDGE_SE2_SEGMENT2D_H

#include "g2o/config.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "vertex_segment2d.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_addons_api.h"

namespace g2o {

  class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2D : public BaseBinaryEdge<4, Vector4D, VertexSE2, VertexSegment2D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2Segment2D();

      Vector2D measurementP1(){ return Eigen::Map<const Vector2D>(&(_measurement[0])); }
      Vector2D measurementP2(){ return Eigen::Map<const Vector2D>(&(_measurement[2])); }
      void  setMeasurementP1(const Vector2D& p1){ Eigen::Map<Vector2D> v(&_measurement[0]); v=p1; }
      void  setMeasurementP2(const Vector2D& p2){ Eigen::Map<Vector2D> v(&_measurement[2]); v=p2; }

      void computeError()
      {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSegment2D* l2 = static_cast<const VertexSegment2D*>(_vertices[1]);
        Eigen::Map<Vector2D> error1(&_error(0));
        Eigen::Map<Vector2D> error2(&_error(2));
        SE2 iEst=v1->estimate().inverse();
        error1 = (iEst * l2->estimateP1());
	error2 = (iEst * l2->estimateP2());
	_error = _error - _measurement;
      }

      virtual bool setMeasurementData(const double* d){
        Eigen::Map<const Vector4D> data(d);
	_measurement = data;
	return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Eigen::Map<Vector4D> data(d);
	data = _measurement;
	return true;
      }

      virtual int measurementDimension() const {return 4;}

      virtual bool setMeasurementFromState(){
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSegment2D* l2 = static_cast<const VertexSegment2D*>(_vertices[1]);
        SE2 iEst=v1->estimate().inverse();
        setMeasurementP1(iEst*l2->estimateP1());
        setMeasurementP2(iEst*l2->estimateP2());
	return true;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);}
/* #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES */
/*       virtual void linearizeOplus(); */
/* #endif */
  };

/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DWriteGnuplotAction: public WriteGnuplotAction { */
/*   public: */
/*     EdgeSE2Segment2DWriteGnuplotAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */

/* #ifdef G2O_HAVE_OPENGL */
/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DDrawAction: public DrawAction{ */
/*   public: */
/*     EdgeSE2Segment2DDrawAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */
/* #endif */

} // end namespace

#endif
