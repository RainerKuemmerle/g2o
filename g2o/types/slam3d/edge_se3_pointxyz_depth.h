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

#ifndef G2O_EDGE_PROJECT_DEPTH_H_
#define G2O_EDGE_PROJECT_DEPTH_H_

#include "g2o/core/base_binary_edge.h"
#include "g2o_types_slam3d_api.h"
#include "parameter_camera.h"
#include "vertex_pointxyz.h"
#include "vertex_se3.h"

namespace g2o {

/*! \class EdgeProjectDepth
 * \brief g2o edge from a track to a depth camera node using a depth measurement
 * (true distance, not disparity)
 */
class G2O_TYPES_SLAM3D_API EdgeSE3PointXYZDepth
    : public BaseBinaryEdge<3, Vector3, VertexSE3, VertexPointXYZ> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PointXYZDepth();
  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

  // return the error estimate as a 3-vector
  void computeError() override;
  // jacobian
  void linearizeOplus() override;

  bool setMeasurementData(const number_t* d) override {
    Eigen::Map<const Vector3> v(d);
    measurement_ = v;
    return true;
  }

  bool getMeasurementData(number_t* d) const override {
    Eigen::Map<Vector3> v(d);
    v = measurement_;
    return true;
  }

  int measurementDimension() const override { return 3; }

  bool setMeasurementFromState() override;

  number_t initialEstimatePossible(const OptimizableGraph::VertexSet& from,
                                   OptimizableGraph::Vertex* to) override {
    (void)to;
    return (from.count(vertices_[0]) == 1 ? 1.0 : -1.0);
  }

  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* to) override;

 private:
  bool resolveCaches() override;
  std::shared_ptr<CacheCamera> cache_;
};

}  // namespace g2o
#endif
