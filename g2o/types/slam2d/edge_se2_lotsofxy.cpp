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

#include "edge_se2_lotsofxy.h"

#include <cassert>
#include <cmath>
#include <vector>

#include "se2.h"
#include "vertex_point_xy.h"
#include "vertex_se2.h"

namespace g2o {

EdgeSE2LotsOfXY::EdgeSE2LotsOfXY() { resize(0); }

void EdgeSE2LotsOfXY::computeError() {
  auto* pose = static_cast<VertexSE2*>(vertexRaw(0));

  int observed_points = measurement_.size() / 2;
  for (int i = 0; i < observed_points; i++) {
    auto* xy = static_cast<VertexPointXY*>(vertexRaw(1 + i));
    Vector2 m = pose->estimate().inverse() * xy->estimate();

    const int index = 2 * i;
    error_[index] = m[0] - measurement_[index];
    error_[index + 1] = m[1] - measurement_[index + 1];
  }
}

void EdgeSE2LotsOfXY::linearizeOplus() {
  const auto* vi = static_cast<const VertexSE2*>(vertexRaw(0));
  const double& x1 = vi->estimate().translation()[0];
  const double& y1 = vi->estimate().translation()[1];
  const double& th1 = vi->estimate().rotation().angle();

  double ct = std::cos(th1);
  double st = std::sin(th1);

  MatrixX Ji;
  unsigned int rows = 2 * (vertices_.size() - 1);
  Ji.resize(rows, 3);
  Ji.fill(0);

  Matrix2 poseRot;  // inverse of the rotation matrix associated to the pose
  poseRot << ct, st, -st, ct;

  Matrix2 minusPoseRot = -poseRot;

  for (unsigned int i = 1; i < vertices_.size(); i++) {
    auto* point = static_cast<g2o::VertexPointXY*>(vertexRaw(i));

    const double& x2 = point->estimate()[0];
    const double& y2 = point->estimate()[1];

    unsigned int index = 2 * (i - 1);

    Ji.block<2, 2>(index, 0) = minusPoseRot;

    Ji(index, 2) = ct * (y2 - y1) + st * (x1 - x2);
    Ji(index + 1, 2) = st * (y1 - y2) + ct * (x1 - x2);

    MatrixX Jj;
    Jj.resize(rows, 2);
    Jj.fill(0);
    Jj.block<2, 2>(index, 0) = poseRot;

    jacobianOplus_[i] = Jj;
  }
  jacobianOplus_[0] = Ji;
}

void EdgeSE2LotsOfXY::initialEstimate(const OptimizableGraph::VertexSet& fixed,
                                      OptimizableGraph::Vertex* toEstimate) {
  (void)toEstimate;

  assert(initialEstimatePossible(fixed, toEstimate) &&
         "Bad vertices specified");

  auto* pose = static_cast<VertexSE2*>(vertexRaw(0));

  int observed_points = measurement_.size() / 2;
  std::vector<bool> estimate_this(observed_points, true);
  for (const auto& it : fixed) {
    for (unsigned int i = 1; i < vertices_.size(); i++) {
      auto* vert = static_cast<VertexPointXY*>(vertexRaw(i));
      if (vert->id() == it->id()) estimate_this[i - 1] = false;
    }
  }

  for (unsigned int i = 1; i < vertices_.size(); i++) {
    if (estimate_this[i - 1]) {
      unsigned int index = 2 * (i - 1);
      Vector2 submeas(measurement_[index], measurement_[index + 1]);
      auto* vert = static_cast<VertexPointXY*>(vertexRaw(i));
      vert->setEstimate(pose->estimate() * submeas);
    }
  }
}

double EdgeSE2LotsOfXY::initialEstimatePossible(
    const OptimizableGraph::VertexSet& fixed,
    OptimizableGraph::Vertex* toEstimate) {
  (void)toEstimate;

  for (const auto& it : fixed) {
    if (vertices_[0]->id() == it->id()) {
      return 1.0;
    }
  }

  return -1.0;
}

bool EdgeSE2LotsOfXY::setMeasurementFromState() {
  auto* pose = static_cast<VertexSE2*>(vertexRaw(0));

  int observed_points = measurement_.size() / 2;
  for (int i = 0; i < observed_points; i++) {
    auto* xy = static_cast<VertexPointXY*>(vertexRaw(1 + i));
    Vector2 m = pose->estimate().inverse() * xy->estimate();

    unsigned int index = 2 * i;
    measurement_[index] = m[0];
    measurement_[index + 1] = m[1];
  }

  return true;
}

}  // end namespace g2o
