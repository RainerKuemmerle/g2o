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

#ifndef G2O_EDGE_SE2_SEGMENT2D_POINTLINE_H
#define G2O_EDGE_SE2_SEGMENT2D_POINTLINE_H

#include "g2o/config.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "vertex_segment2d.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_addons_api.h"

namespace g2o {

  class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DPointLine : public BaseBinaryEdge<3, Vector3, VertexSE2, VertexSegment2D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2Segment2DPointLine();

      number_t   theta() const {return _measurement[2];}
      Vector2 point()   const {Eigen::Map<const Vector2> p(&_measurement[0]); return p;}

      void   setTheta(number_t t)  {_measurement[2] = t;}
      void   setPoint(const Vector2& p_)  {Eigen::Map<Vector2> p(&_measurement[0]); p=p_; }

      int pointNum() const {return _pointNum;}
      void setPointNum(int pn) {_pointNum = pn;}

      void computeError()
      {
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSegment2D* l2 = static_cast<const VertexSegment2D*>(_vertices[1]);
        SE2 iEst=v1->estimate().inverse();
        Vector2 predP1 = iEst * l2->estimateP1();
        Vector2 predP2 = iEst * l2->estimateP2();
        Vector2 dP = predP2 - predP1;
        Vector2 normal(dP.y(), -dP.x()); normal.normalize();
        Vector3 prediction;
        prediction [2] = std::atan2(normal.y(), normal.x());
        Eigen::Map<Vector2> pt(&prediction[0]);
        pt = (_pointNum==0) ? predP1 : predP2;
        _error = prediction - _measurement;
        _error[2]=normalize_theta(_error[2]);
      }

      virtual bool setMeasurementData(const number_t* d) {
        Eigen::Map<const Vector3> data(d);
        _measurement = data;
        return true;
      }

      virtual bool getMeasurementData(number_t* d) const {
        Eigen::Map<Vector3> data(d);
        data = _measurement;
        return true;
      }

      virtual int measurementDimension() const {return 3;}

      virtual bool setMeasurementFromState(){
        const VertexSE2* v1 = static_cast<const VertexSE2*>(_vertices[0]);
        const VertexSegment2D* l2 = static_cast<const VertexSegment2D*>(_vertices[1]);
        SE2 iEst=v1->estimate().inverse();
        Vector2 predP1 = iEst * l2->estimateP1();
        Vector2 predP2 = iEst * l2->estimateP2();
        Vector2 dP = predP2 - predP1;
        Vector2 normal(dP.y(), -dP.x()); normal.normalize();
        Vector3 prediction;
        prediction [2] = std::atan2(normal.y(), normal.x());
        Eigen::Map<Vector2> pt(&prediction[0]);
        pt = (_pointNum==0)?predP1:predP2;
        setMeasurement(prediction);
        return true;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

  protected:
      int _pointNum;

/* #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES */
/*       virtual void linearizeOplus(); */
/* #endif */
  };

/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DPointLineWriteGnuplotAction: public WriteGnuplotAction { */
/*   public: */
/*     EdgeSE2Segment2DPointLineWriteGnuplotAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */

/* #ifdef G2O_HAVE_OPENGL */
/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DPointLineDrawAction: public DrawAction{ */
/*   public: */
/*     EdgeSE2Segment2DPointLineDrawAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */
/* #endif */

} // end namespace

#endif
