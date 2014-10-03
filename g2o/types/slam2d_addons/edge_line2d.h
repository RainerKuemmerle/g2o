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

#ifndef G2O_EDGE_LINE2D_H
#define G2O_EDGE_LINE2D_H

#include "vertex_line2d.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "types_slam2d_addons.h"

namespace g2o {

  class G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2D : public BaseBinaryEdge<2, Line2D, VertexLine2D, VertexLine2D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeLine2D();

      void computeError()
      {
        const VertexLine2D* v1 = static_cast<const VertexLine2D*>(_vertices[0]);
        const VertexLine2D* v2 = static_cast<const VertexLine2D*>(_vertices[1]);
        _error = (v2->estimate()-v1->estimate())-_measurement;
      }
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual void setMeasurement(const Line2D& m){
        _measurement = m;
      }

      virtual void setMeasurement(const Vector2D& m){
        _measurement = m;
      }

      virtual bool setMeasurementData(const double* d){
	Eigen::Map<const Vector2D> m(d);
        _measurement=Line2D(m);
        return true;
      }

      virtual bool getMeasurementData(double* d) const {
	Eigen::Map<Vector2D> m(d);
	m=_measurement;
        return true;
      }

      virtual int measurementDimension() const {return 2;}

      virtual bool setMeasurementFromState() {
        const VertexLine2D* v1 = static_cast<const VertexLine2D*>(_vertices[0]);
        const VertexLine2D* v2 = static_cast<const VertexLine2D*>(_vertices[1]);
        _measurement = Line2D((Vector2D)(v2->estimate())-(Vector2D)v1->estimate());
        return true;
      }


      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& , OptimizableGraph::Vertex* ) { return 0.;}
#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
      virtual void linearizeOplus();
#endif
  };

}
#endif
