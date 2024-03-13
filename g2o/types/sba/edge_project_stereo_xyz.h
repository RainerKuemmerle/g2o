// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#ifndef G2O_SBA_EDGEPROJECTSTEREOXYZ_H
#define G2O_SBA_EDGEPROJECTSTEREOXYZ_H

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o_types_sba_api.h"
#include "vertex_se3_expmap.h"

namespace g2o {

// Projection using focal_length in x and y directions stereo
class G2O_TYPES_SBA_API EdgeStereoSE3ProjectXYZ
    : public BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexSE3Expmap> {
 public:
  void computeError() override {
    const VertexSE3Expmap* v1 = vertexXnRaw<1>();
    const VertexPointXYZ* v2 = vertexXnRaw<0>();
    Vector3 obs(measurement_);
    error_ = obs - cam_project(v1->estimate().map(v2->estimate()), bf);
  }

  bool isDepthPositive() {
    const VertexSE3Expmap* v1 = vertexXnRaw<1>();
    const VertexPointXYZ* v2 = vertexXnRaw<0>();
    return (v1->estimate().map(v2->estimate()))(2) > 0;
  }

  void linearizeOplus() override;

  [[nodiscard]] Vector3 cam_project(const Vector3& trans_xyz,
                                    const float& bf) const;

  double fx = 1., fy = 1., cx = 0.5, cy = 0.5, bf = 0;
};

}  // namespace g2o

#endif
