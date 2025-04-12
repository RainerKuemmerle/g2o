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

#ifndef G2O_EDGE_XY_PRIOR_H
#define G2O_EDGE_XY_PRIOR_H

#include "g2o/config.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o_types_slam2d_api.h"
#include "vertex_point_xy.h"

namespace g2o {

class G2O_TYPES_SLAM2D_API EdgeXYPrior
    : public BaseUnaryEdge<2, Vector2, VertexPointXY> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeXYPrior();

  void computeError() {
    const VertexPointXY* v = static_cast<const VertexPointXY*>(_vertices[0]);
    _error = v->estimate() - _measurement;
  }
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual void setMeasurement(const Vector2& m) { _measurement = m; }

  virtual bool setMeasurementData(const double* d) {
    _measurement = Vector2(d[0], d[1]);
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector2> m(d);
    m = _measurement;
    return true;
  }

  virtual int measurementDimension() const { return 2; }

  virtual bool setMeasurementFromState() {
    const VertexPointXY* v = static_cast<const VertexPointXY*>(_vertices[0]);
    _measurement = v->estimate();
    return true;
  }

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                         OptimizableGraph::Vertex*) {
    return 0.;
  }
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  virtual void linearizeOplus();
#endif
};

}  // namespace g2o

#endif
