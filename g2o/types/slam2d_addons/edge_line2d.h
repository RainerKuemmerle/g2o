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

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "types_slam2d_addons.h"
#include "vertex_line2d.h"

namespace g2o {

class G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2D
    : public BaseBinaryEdge<2, Line2D, VertexLine2D, VertexLine2D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeLine2D();

  void computeError() override {
    const VertexLine2D* v1 = vertexXnRaw<0>();
    const VertexLine2D* v2 = vertexXnRaw<1>();
    error_ = (v2->estimate() - v1->estimate()) - measurement_;
  }
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void setMeasurement(const Line2D& m) override { measurement_ = m; }

  virtual void setMeasurement(const Vector2& m) { measurement_ = Line2D(m); }

  bool setMeasurementData(const number_t* d) override {
    Eigen::Map<const Vector2> m(d);
    measurement_ = Line2D(m);
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    Eigen::Map<Vector2> m(d);
    m = measurement_;
    return true;
  }

  int measurementDimension() const override { return 2; }

  bool setMeasurementFromState() override {
    const VertexLine2D* v1 = vertexXnRaw<0>();
    const VertexLine2D* v2 = vertexXnRaw<1>();
    measurement_ = Line2D(Vector2(v2->estimate()) - Vector2(v1->estimate()));
    return true;
  }

  number_t initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                   OptimizableGraph::Vertex*) override {
    return 0;
  }
  void linearizeOplus() override;
};

}  // namespace g2o
#endif
