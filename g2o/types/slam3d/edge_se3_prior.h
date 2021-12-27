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

#ifndef G2O_EDGE_SE3_PRIOR_H_
#define G2O_EDGE_SE3_PRIOR_H_

#include <memory>

#include "g2o/core/base_unary_edge.h"
#include "g2o_types_slam3d_api.h"
#include "parameter_se3_offset.h"
#include "vertex_se3.h"

namespace g2o {
/**
 * \brief prior for an SE3 element
 *
 * Provides a prior for a 3d pose vertex. Again the measurement is represented
 * by an Isometry3 matrix.
 */
class G2O_TYPES_SLAM3D_API EdgeSE3Prior
    : public BaseUnaryEdge<6, Isometry3, VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3Prior();
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  // return the error estimate as a 3-vector
  void computeError() override;

  // jacobian
  void linearizeOplus() override;

  void setMeasurement(const Isometry3& m) override {
    measurement_ = m;
    inverseMeasurement_ = m.inverse();
  }

  bool setMeasurementData(const number_t* d) override {
    Eigen::Map<const Vector7> v(d);
    measurement_ = internal::fromVectorQT(v);
    inverseMeasurement_ = measurement_.inverse();
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    Eigen::Map<Vector7> v(d);
    v = internal::toVectorQT(measurement_);
    return true;
  }

  int measurementDimension() const override { return 7; }

  bool setMeasurementFromState() override;

  number_t initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/,
                                   OptimizableGraph::Vertex* /*to*/) override {
    return 1.;
  }

  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* to) override;

 protected:
  Isometry3 inverseMeasurement_;
  bool resolveCaches() override;
  std::shared_ptr<CacheSE3Offset> cache_;
};

}  // namespace g2o
#endif
