// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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

#include "edge_sba_scale.h"

#include <Eigen/Core>

#include "g2o/core/eigen_types.h"
#include "g2o/types/sba/sbacam.h"
#include "g2o/types/sba/vertex_cam.h"
#include "g2o/types/slam3d/se3quat.h"

namespace g2o {

void EdgeSBAScale::initialEstimate(const OptimizableGraph::VertexSet& from_,
                                   OptimizableGraph::Vertex* /*to_*/) {
  auto v1 = vertexXn<0>();
  auto v2 = vertexXn<1>();
  // compute the translation vector of v1 w.r.t v2
  if (from_.count(v1) == 1) {
    SE3Quat delta = (v1->estimate().inverse() * v2->estimate());
    double norm = delta.translation().norm();
    double alpha = measurement_ / norm;
    delta.setTranslation(delta.translation() * alpha);
    v2->setEstimate(SBACam(v1->estimate() * delta));
  } else {
    SE3Quat delta = (v2->estimate().inverse() * v1->estimate());
    double norm = delta.translation().norm();
    double alpha = measurement_ / norm;
    delta.setTranslation(delta.translation() * alpha);
    v1->setEstimate(SBACam(v2->estimate() * delta));
  }
}

void EdgeSBAScale::computeError() {
  const VertexCam* v1 = vertexXnRaw<0>();
  const VertexCam* v2 = vertexXnRaw<1>();
  Vector3 dt = v2->estimate().translation() - v1->estimate().translation();
  error_[0] = measurement_ - dt.norm();
}

}  // namespace g2o
