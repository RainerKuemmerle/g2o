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

#ifndef G2O_EDGE_POINTXYZ_H
#define G2O_EDGE_POINTXYZ_H

#include <Eigen/Core>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o_types_slam3d_api.h"
#include "vertex_pointxyz.h"

namespace g2o {

class G2O_TYPES_SLAM3D_API EdgePointXYZ
    : public BaseBinaryEdge<3, Vector3, VertexPointXYZ, VertexPointXYZ> {
 public:
  EdgePointXYZ();

  void computeError() override {
    const VertexPointXYZ* v1 = vertexXnRaw<0>();
    const VertexPointXYZ* v2 = vertexXnRaw<1>();
    error_ = (v2->estimate() - v1->estimate()) - measurement_;
  }

  void setMeasurement(const Vector3& m) override { measurement_ = m; }

  bool setMeasurementFromState() override {
    const VertexPointXYZ* v1 = vertexXnRaw<0>();
    const VertexPointXYZ* v2 = vertexXnRaw<1>();
    measurement_ = v2->estimate() - v1->estimate();
    return true;
  }

  double initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                 OptimizableGraph::Vertex*) override {
    return 0;
  }
#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
  void linearizeOplus() override;
#endif
};

}  // namespace g2o

#endif
