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

#ifndef G2O_EDGE_SE2_SEGMENT2D_LINE_H
#define G2O_EDGE_SE2_SEGMENT2D_LINE_H

#include "g2o/config.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "vertex_segment2d.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_addons_api.h"

namespace g2o {

  class EdgeSE2Segment2DLine : public BaseBinaryEdge<2, Vector2D, VertexSE2, VertexSegment2D> //Avoid redefinition of BaseEdge in MSVC
  {
    public:
      G2O_TYPES_SLAM2D_ADDONS_API EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DLine();

      G2O_TYPES_SLAM2D_ADDONS_API double theta() const {return _measurement[0];}
      G2O_TYPES_SLAM2D_ADDONS_API double rho()   const {return _measurement[1];}

      G2O_TYPES_SLAM2D_ADDONS_API void   setTheta( double t)  {_measurement[0] = t;}
      G2O_TYPES_SLAM2D_ADDONS_API void   setRho( double r)    {_measurement[1] = r;}


      G2O_TYPES_SLAM2D_ADDONS_API void computeError()
      {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSegment2D* l2 = static_cast<const VertexSegment2D*>(_vertices[1]);
        SE2 iEst=v1->estimate().inverse();
        Vector2D predP1 = iEst * l2->estimateP1();
        Vector2D predP2 = iEst * l2->estimateP2();
        Vector2D dP = predP2 - predP1;
        Vector2D normal(dP.y(), -dP.x()); normal.normalize();
        Vector2D prediction(atan2(normal.y(), normal.x()),
			    predP1.dot(normal)*.5 + predP2.dot(normal)*.5);

	_error=prediction-_measurement;
        _error[0]=normalize_theta(_error[0]);
      }

      G2O_TYPES_SLAM2D_ADDONS_API virtual bool setMeasurementData(const double* d){
        Eigen::Map<const Vector2D> data(d);
	_measurement = data;
	return true;
      }

      G2O_TYPES_SLAM2D_ADDONS_API virtual bool getMeasurementData(double* d) const{
        Eigen::Map<Vector2D> data(d);
	data = _measurement;
	return true;
      }

      G2O_TYPES_SLAM2D_ADDONS_API virtual int measurementDimension() const {return 2;}

      G2O_TYPES_SLAM2D_ADDONS_API virtual bool setMeasurementFromState(){
     const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSegment2D* l2 = static_cast<const VertexSegment2D*>(_vertices[1]);
        SE2 iEst=v1->estimate().inverse();
        Vector2D predP1 = iEst * l2->estimateP1();
        Vector2D predP2 = iEst * l2->estimateP2();
        Vector2D dP = predP2 - predP1;
        Vector2D normal(dP.y(), -dP.x()); normal.normalize();
        Vector2D prediction(atan2(normal.y(), normal.x()),
			    predP1.dot(normal)*.5 + predP2.dot(normal)*.5);
	_measurement = prediction;
	return true;
      }

      G2O_TYPES_SLAM2D_ADDONS_API virtual bool read(std::istream& is);
      G2O_TYPES_SLAM2D_ADDONS_API virtual bool write(std::ostream& os) const;


/* #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES */
/*       virtual void linearizeOplus(); */
/* #endif */
  };

/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DLineWriteGnuplotAction: public WriteGnuplotAction { */
/*   public: */
/*     EdgeSE2Segment2DLineWriteGnuplotAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */

/* #ifdef G2O_HAVE_OPENGL */
/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DLineDrawAction: public DrawAction{ */
/*   public: */
/*     EdgeSE2Segment2DLineDrawAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */
/* #endif */

} // end namespace

#endif
