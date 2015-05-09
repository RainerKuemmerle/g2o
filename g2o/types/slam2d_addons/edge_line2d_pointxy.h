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

#ifndef G2O_EDGE_LINE2D_POINTXY_H
#define G2O_EDGE_LINE2D_POINTXY_H

#include "g2o/config.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "vertex_line2d.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/stuff/misc.h"
#include "g2o_types_slam2d_addons_api.h"

namespace g2o {

  class EdgeLine2DPointXY : public BaseBinaryEdge<1, double, VertexLine2D, VertexPointXY> //Avoid redefinition of BaseEdge in MSVC
  {
    public:
      G2O_TYPES_SLAM2D_ADDONS_API EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2DPointXY();

      G2O_TYPES_SLAM2D_ADDONS_API void computeError()
      {
        const VertexLine2D* l = static_cast<const VertexLine2D*>(_vertices[0]);
        const VertexPointXY* p = static_cast<const VertexPointXY*>(_vertices[1]);
        Vector2D n(cos(l->theta()), sin(l->theta()));
        double prediction=n.dot(p->estimate())-l->rho();
        _error[0] =  prediction - _measurement;
      }

      G2O_TYPES_SLAM2D_ADDONS_API virtual bool setMeasurementData(const double* d){
	_measurement = *d;
        return true;
      }

      G2O_TYPES_SLAM2D_ADDONS_API virtual bool getMeasurementData(double* d) const{
        *d = _measurement;
        return true;
      }

      G2O_TYPES_SLAM2D_ADDONS_API virtual int measurementDimension() const {return 1;}

      G2O_TYPES_SLAM2D_ADDONS_API virtual bool setMeasurementFromState(){
        const VertexLine2D* l = static_cast<const VertexLine2D*>(_vertices[0]);
        const VertexPointXY* p = static_cast<const VertexPointXY*>(_vertices[1]);
        Vector2D n(cos(l->theta()), sin(l->theta()));
        double prediction=n.dot(p->estimate())-l->rho();
	_measurement = prediction;
        return true;
      }

      G2O_TYPES_SLAM2D_ADDONS_API virtual bool read(std::istream& is);
      G2O_TYPES_SLAM2D_ADDONS_API virtual bool write(std::ostream& os) const;

      /* virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to); */
      /* virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);} */
/* #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES */
/*       virtual void linearizeOplus(); */
/* #endif */
  };

/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2DPointXYWriteGnuplotAction: public WriteGnuplotAction { */
/*   public: */
/*     EdgeLine2DPointXYWriteGnuplotAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */

/* #ifdef G2O_HAVE_OPENGL */
/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2DPointXYDrawAction: public DrawAction{ */
/*   public: */
/*     EdgeLine2DPointXYDrawAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */
/* #endif */

} // end namespace

#endif
