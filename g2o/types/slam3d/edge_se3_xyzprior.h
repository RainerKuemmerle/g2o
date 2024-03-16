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

#ifndef G2O_EDGE_SE3_PRIOR_XYZ_H
#define G2O_EDGE_SE3_PRIOR_XYZ_H

#include <memory>

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o_types_slam3d_api.h"
#include "parameter_se3_offset.h"
#include "vertex_se3.h"

namespace g2o {
class CacheSE3Offset;

/**
 * \brief Prior for a 3D pose with constraints only in xyz direction
 */
class G2O_TYPES_SLAM3D_API EdgeSE3XYZPrior
    : public BaseUnaryEdge<3, Vector3, VertexSE3> {
 public:
  EdgeSE3XYZPrior();

  void computeError() override;
  void linearizeOplus() override;
  bool setMeasurementFromState() override;

  double initialEstimatePossible(const OptimizableGraph::VertexSet& /*from*/,
                                 OptimizableGraph::Vertex* /*to*/) override {
    return 1.;
  }
  void initialEstimate(const OptimizableGraph::VertexSet& /*from_*/,
                       OptimizableGraph::Vertex* /*to_*/) override;

 protected:
  bool resolveCaches() override;
  std::shared_ptr<CacheSE3Offset> cache_;
};

}  // namespace g2o

#endif
