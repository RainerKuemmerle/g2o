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

#ifndef G2O_EDGE_SE2_PRIOR_H
#define G2O_EDGE_SE2_PRIOR_H

#include "g2o/core/base_unary_edge.h"
#include "g2o_types_slam2d_api.h"
#include "vertex_se2.h"

namespace g2o {

/**
 * \brief Prior for a two D pose
 */
class G2O_TYPES_SLAM2D_API EdgeSE2Prior
    : public BaseUnaryEdge<3, SE2, VertexSE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE2Prior() = default;

  void computeError() override {
    const VertexSE2* v1 = vertexXnRaw<0>();
    SE2 delta = inverseMeasurement_ * v1->estimate();
    error_ = delta.toVector();
  }

  void linearizeOplus() override {
    jacobianOplusXi_.setZero();
    jacobianOplusXi_.block<2, 2>(0, 0) =
        inverseMeasurement_.rotation().toRotationMatrix();
    jacobianOplusXi_(2, 2) = 1.;
  }

  void setMeasurement(const SE2& m) override;
  bool setMeasurementData(const number_t* d) override;

  bool getMeasurementData(number_t* d) const override {
    Eigen::Map<Vector3> v(d);
    v = measurement_.toVector();
    return true;
  }

  int measurementDimension() const override { return 3; }

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  number_t initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                   OptimizableGraph::Vertex*) override {
    return 1.;
  }
  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* to) override;

 protected:
  SE2 inverseMeasurement_;
};

}  // namespace g2o

#endif
