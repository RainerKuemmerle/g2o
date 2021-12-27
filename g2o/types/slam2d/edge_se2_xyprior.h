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

#ifndef G2O_EDGE_SE2_PRIOR_XY_H
#define G2O_EDGE_SE2_PRIOR_XY_H

#include "g2o/core/base_unary_edge.h"
#include "g2o_types_slam2d_api.h"
#include "vertex_se2.h"

namespace g2o {

/**
 * \brief Prior for a two D pose with constraints only in xy direction (like
 * gps)
 */
class G2O_TYPES_SLAM2D_API EdgeSE2XYPrior
    : public BaseUnaryEdge<2, Vector2, VertexSE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE2XYPrior() = default;

  bool setMeasurementData(const number_t* d) override {
    measurement_[0] = d[0];
    measurement_[1] = d[1];
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    d[0] = measurement_[0];
    d[1] = measurement_[1];
    return true;
  }

  int measurementDimension() const override { return 2; }

  void linearizeOplus() override;

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void computeError() override {
    const VertexSE2* v = vertexXnRaw<0>();
    error_ = v->estimate().translation() - measurement_;
  }
};

}  // namespace g2o

#endif
