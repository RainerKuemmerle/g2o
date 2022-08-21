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

#ifndef G2O_EDGE_SE2_H
#define G2O_EDGE_SE2_H

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_api.h"
#include "vertex_se2.h"

namespace g2o {

/**
 * \brief 2D edge between two Vertex2
 */
class G2O_TYPES_SLAM2D_API EdgeSE2
    : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE2() = default;

  void computeError() override {
    const VertexSE2* v1 = vertexXnRaw<0>();
    const VertexSE2* v2 = vertexXnRaw<1>();
    SE2 delta =
        inverseMeasurement_ * (v1->estimate().inverse() * v2->estimate());
    error_ = delta.toVector();
  }
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void setMeasurement(const SE2& m) override {
    measurement_ = m;
    inverseMeasurement_ = m.inverse();
  }

  bool setMeasurementData(const number_t* d) override {
    measurement_ = SE2(d[0], d[1], d[2]);
    inverseMeasurement_ = measurement_.inverse();
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    Vector3 v = measurement_.toVector();
    d[0] = v[0];
    d[1] = v[1];
    d[2] = v[2];
    return true;
  }

  int measurementDimension() const override { return 3; }

  bool setMeasurementFromState() override {
    const VertexSE2* v1 = vertexXnRaw<0>();
    const VertexSE2* v2 = vertexXnRaw<1>();
    measurement_ = v1->estimate().inverse() * v2->estimate();
    inverseMeasurement_ = measurement_.inverse();
    return true;
  }

  number_t initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                   OptimizableGraph::Vertex*) override {
    return 1.;
  }
  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* to) override;
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  void linearizeOplus() override;
#endif
 protected:
  SE2 inverseMeasurement_;
};

class G2O_TYPES_SLAM2D_API EdgeSE2WriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  EdgeSE2WriteGnuplotAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_API EdgeSE2DrawAction : public DrawAction {
 public:
  EdgeSE2DrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;

 protected:
  bool refreshPropertyPtrs(
      const std::shared_ptr<HyperGraphElementAction::Parameters>& params_)
      override;
  std::shared_ptr<FloatProperty> triangleX_, triangleY_;
};
#endif

}  // namespace g2o

#endif
