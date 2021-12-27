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

#ifndef G2O_EDGE_SE2_OFFSET_H_
#define G2O_EDGE_SE2_OFFSET_H_

#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam2d_api.h"
#include "vertex_se2.h"

namespace g2o {

class CacheSE2Offset;

/**
 * \brief Offset edge
 */
// first two args are the measurement type, second two the connection classes
class G2O_TYPES_SLAM2D_API EdgeSE2Offset
    : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE2Offset();
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  void computeError() override;

  // jacobian
  // virtual void linearizeOplus();

  void setMeasurement(const SE2& m) override {
    measurement_ = m;
    inverseMeasurement_ = m.inverse();
  }

  bool setMeasurementData(const number_t* d) override {
    Eigen::Map<const Vector3> v(d);
    measurement_.fromVector(v);
    inverseMeasurement_ = measurement_.inverse();
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    Eigen::Map<Vector3> v(d);
    v = measurement_.toVector();
    return true;
  }

  int measurementDimension() const override { return 3; }

  bool setMeasurementFromState() override;

  number_t initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/,
                                   OptimizableGraph::Vertex* /*to*/) override {
    return 1.;
  }

  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* to) override;

 protected:
  SE2 inverseMeasurement_;
  bool resolveCaches() override;
  std::shared_ptr<CacheSE2Offset> cacheFrom_, cacheTo_;
};

}  // namespace g2o
#endif
