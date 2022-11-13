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

#ifndef G2O_TYPES_SLAM3D_ONLINE_H
#define G2O_TYPES_SLAM3D_ONLINE_H

#include <iostream>

#include "g2o/types/slam3d/edge_se3.h"
#include "g2o_interactive_api.h"

namespace g2o {

class G2O_INTERACTIVE_API OnlineVertexSE3 : public VertexSE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  OnlineVertexSE3() : updatedEstimate(Eigen::Isometry3d::Identity()) {}

  void oplusImpl(const VectorX::MapType& update) override {
    VertexSE3::oplusImpl(update);
    updatedEstimate = estimate_;
  }

  void oplusUpdatedEstimate(double* update) {
    Eigen::Map<const Vector6> v(update);
    Isometry3 increment = internal::fromVectorMQT(v);
    updatedEstimate = estimate_ * increment;
  }

  VertexSE3::EstimateType updatedEstimate;
};

class G2O_INTERACTIVE_API OnlineEdgeSE3 : public EdgeSE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  OnlineEdgeSE3() = default;

  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* /* to */) override {
    auto fromEdge = std::static_pointer_cast<OnlineVertexSE3>(vertexXn<0>());
    auto toEdge = std::static_pointer_cast<OnlineVertexSE3>(vertexXn<1>());
    if (from.count(fromEdge) > 0) {
      toEdge->updatedEstimate = fromEdge->updatedEstimate * measurement_;
      toEdge->setEstimate(toEdge->updatedEstimate);
    } else {
      fromEdge->updatedEstimate = toEdge->updatedEstimate * inverseMeasurement_;
      fromEdge->setEstimate(fromEdge->updatedEstimate);
    }
  }

  double chi2() const override {
    auto* from = static_cast<OnlineVertexSE3*>(vertexXnRaw<0>());
    auto* to = static_cast<OnlineVertexSE3*>(vertexXnRaw<1>());
    Eigen::Isometry3d delta =
        inverseMeasurement_ * from->estimate().inverse() * to->estimate();
    Vector6 error = internal::toVectorMQT(delta);
    return error.dot(information() * error);
  }
};

}  // namespace g2o

#endif
