// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#include "edge_se3_lotsofxyz.h"

#include <Eigen/Geometry>
#include <cassert>
#include <vector>

#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"

namespace g2o {

EdgeSE3LotsOfXYZ::EdgeSE3LotsOfXYZ() { resize(0); }

bool EdgeSE3LotsOfXYZ::setMeasurementFromState() {
  auto* pose = static_cast<VertexSE3*>(vertexRaw(0));

  Eigen::Transform<double, 3, 1> poseinv = pose->estimate().inverse();

  int observed_points = measurement_.size() / 3;
  for (int i = 0; i < observed_points; i++) {
    auto* xyz = static_cast<VertexPointXYZ*>(vertexRaw(1 + i));
    //      const Vector3 &pt = xyz->estimate();
    Vector3 m = poseinv * xyz->estimate();

    const int index = 3 * i;
    measurement_[index] = m[0];
    measurement_[index + 1] = m[1];
    measurement_[index + 2] = m[2];
  }
  return true;
}

void EdgeSE3LotsOfXYZ::computeError() {
  auto* pose = static_cast<VertexSE3*>(vertexRaw(0));

  int observed_points = measurement_.size() / 3;
  for (int i = 0; i < observed_points; i++) {
    auto* xyz = static_cast<VertexPointXYZ*>(vertexRaw(1 + i));
    Vector3 m = pose->estimate().inverse() * xyz->estimate();

    const int index = 3 * i;
    error_[index] = m[0] - measurement_[index];
    error_[index + 1] = m[1] - measurement_[index + 1];
    error_[index + 2] = m[2] - measurement_[index + 2];
  }
}

void EdgeSE3LotsOfXYZ::linearizeOplus() {
  auto* pose = static_cast<g2o::VertexSE3*>(vertexRaw(0));

  // initialize Ji matrix
  MatrixX Ji;
  unsigned int rows = 3 * (vertices_.size() - 1);
  Ji.resize(rows, 6);
  Ji.fill(0);

  Matrix3 poseRot = pose->estimate().inverse().rotation();

  for (unsigned int i = 1; i < vertices_.size(); i++) {
    auto* point = static_cast<g2o::VertexPointXYZ*>(vertexRaw(i));
    Vector3 Zcam = pose->estimate().inverse() * point->estimate();

    unsigned int index = 3 * (i - 1);

    // Ji.block<3,3>(index,0) = -poseRot;
    Ji.block<3, 3>(index, 0) = -Matrix3::Identity();

    Ji(index, 3) = -0.0;
    Ji(index, 4) = -2 * Zcam(2);
    Ji(index, 5) = 2 * Zcam(1);

    Ji(index + 1, 3) = 2 * Zcam(2);
    Ji(index + 1, 4) = -0.0;
    Ji(index + 1, 5) = -2 * Zcam(0);

    Ji(index + 2, 3) = -2 * Zcam(1);
    Ji(index + 2, 4) = 2 * Zcam(0);
    Ji(index + 2, 5) = -0.0;

    MatrixX Jj;
    Jj.resize(rows, 3);
    Jj.fill(0);
    Jj.block<3, 3>(index, 0) = poseRot;

    jacobianOplus_[i] = Jj;
  }

  jacobianOplus_[0] = Ji;
}

void EdgeSE3LotsOfXYZ::initialEstimate(const OptimizableGraph::VertexSet& fixed,
                                       OptimizableGraph::Vertex* toEstimate) {
  (void)toEstimate;

  assert(initialEstimatePossible(fixed, toEstimate) &&
         "Bad vertices specified");

  auto* pose = static_cast<VertexSE3*>(vertexRaw(0));

  int observed_points = measurement_.size() / 3;
  std::vector<bool> estimate_this(observed_points, true);
  for (const auto& it : fixed) {
    for (unsigned int i = 1; i < vertices_.size(); i++) {
      auto* vert = static_cast<VertexPointXYZ*>(vertexRaw(i));
      if (vert->id() == it->id()) estimate_this[i - 1] = false;
    }
  }

  for (unsigned int i = 1; i < vertices_.size(); i++) {
    if (estimate_this[i - 1]) {
      unsigned int index = 3 * (i - 1);
      Vector3 submeas(measurement_[index], measurement_[index + 1],
                      measurement_[index + 2]);
      auto* vert = static_cast<VertexPointXYZ*>(vertexRaw(i));
      vert->setEstimate(pose->estimate() * submeas);
    }
  }
}

double EdgeSE3LotsOfXYZ::initialEstimatePossible(
    const OptimizableGraph::VertexSet& fixed,
    OptimizableGraph::Vertex* toEstimate) {
  (void)toEstimate;

  for (const auto& it : fixed) {
    if (vertexRaw(0)->id() == it->id()) {
      return 1.0;
    }
  }

  return -1.0;
}

}  // namespace g2o
