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

#ifndef G2O_EDGE_SE3_POINT_XYZ_H_
#define G2O_EDGE_SE3_POINT_XYZ_H_

#include <Eigen/Core>
#include <memory>

#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o_types_slam3d_api.h"
#include "parameter_se3_offset.h"
#include "vertex_pointxyz.h"
#include "vertex_se3.h"

namespace g2o {
class CacheSE3Offset;

/*! \class EdgeSE3PointXYZ
 * \brief g2o edge from a track to a point node
 */
// first two args are the measurement type, second two the connection classes
class G2O_TYPES_SLAM3D_API EdgeSE3PointXYZ
    : public BaseBinaryEdge<3, Vector3, VertexSE3, VertexPointXYZ> {
 public:
  EdgeSE3PointXYZ();

  // return the error estimate as a 3-vector
  void computeError() override;
  // jacobian
  void linearizeOplus() override;

  void setMeasurement(const Vector3& m) override { measurement_ = m; }

  bool setMeasurementFromState() override;

  double initialEstimatePossible(const OptimizableGraph::VertexSet& from,
                                 OptimizableGraph::Vertex* to) override {
    (void)to;
    return (from.count(vertices_[0]) == 1 ? 1.0 : -1.0);
  }

  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* to) override;

 private:
  std::shared_ptr<CacheSE3Offset> cache_;
  bool resolveCaches() override;
};

#ifdef G2O_HAVE_OPENGL
class EdgeSE3PointXYZDrawAction : public DrawAction {
 public:
  EdgeSE3PointXYZDrawAction();
  bool operator()(HyperGraph::HyperGraphElement& element,
                  const std::shared_ptr<HyperGraphElementAction::Parameters>&
                      params_) override;
};
#endif

}  // namespace g2o
#endif
