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

  void computeError() override {
    error_ = vertexXnRaw<0>()->estimate() - measurement_;
  }
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void setMeasurement(const Vector2& m) override { measurement_ = m; }

  bool setMeasurementData(const number_t* d) override {
    measurement_ = Vector2(d[0], d[1]);
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    Eigen::Map<Vector2> m(d);
    m = measurement_;
    return true;
  }

  int measurementDimension() const override { return 2; }

  bool setMeasurementFromState() override {
    measurement_ = vertexXnRaw<0>()->estimate();
    return true;
  }

  number_t initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                   OptimizableGraph::Vertex*) override {
    return 0.;
  }
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void linearizeOplus() override;
#endif
};

}  // namespace g2o

#endif
