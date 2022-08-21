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

#ifndef G2O_EDGE_SE2_POINT_XY_BEARING_H
#define G2O_EDGE_SE2_POINT_XY_BEARING_H

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_api.h"
#include "vertex_point_xy.h"
#include "vertex_se2.h"

namespace g2o {

class G2O_TYPES_SLAM2D_API EdgeSE2PointXYBearing
    : public BaseBinaryEdge<1, number_t, VertexSE2, VertexPointXY> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE2PointXYBearing() = default;
  void computeError() override {
    const VertexSE2* v1 = vertexXnRaw<0>();
    const VertexPointXY* l2 = vertexXnRaw<1>();
    Vector2 delta = (v1->estimate().inverse() * l2->estimate());
    number_t angle = std::atan2(delta[1], delta[0]);
    error_[0] = normalize_theta(measurement_ - angle);
  }

  bool setMeasurementData(const number_t* d) override {
    measurement_ = d[0];
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    d[0] = measurement_;
    return true;
  }

  int measurementDimension() const override { return 1; }

  bool setMeasurementFromState() override {
    const VertexSE2* v1 = vertexXnRaw<0>();
    const VertexPointXY* l2 = vertexXnRaw<1>();
    Vector2 delta = (v1->estimate().inverse() * l2->estimate());
    measurement_ = std::atan2(delta[1], delta[0]);
    return true;
  }

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  number_t initialEstimatePossible(const OptimizableGraph::VertexSet& from,
                                   OptimizableGraph::Vertex*) override {
    return (from.count(vertices_[0]) == 1 ? 1.0 : -1.0);
  }
  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* to) override;
};

class G2O_TYPES_SLAM2D_API EdgeSE2PointXYBearingWriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  EdgeSE2PointXYBearingWriteGnuplotAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_API EdgeSE2PointXYBearingDrawAction : public DrawAction {
 public:
  EdgeSE2PointXYBearingDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;
};
#endif

}  // namespace g2o

#endif
