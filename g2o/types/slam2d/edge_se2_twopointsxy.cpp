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

#include "edge_se2_twopointsxy.h"

#include <Eigen/Core>
#include <cassert>

#include "vertex_point_xy.h"
#include "vertex_se2.h"

namespace g2o {

void EdgeSE2TwoPointsXY::computeError() {
  auto* pose = vertexXnRaw<0>();
  auto* xy1 = vertexXnRaw<1>();
  auto* xy2 = vertexXnRaw<2>();

  Vector2 m1 = pose->estimate().inverse() * xy1->estimate();
  Vector2 m2 = pose->estimate().inverse() * xy2->estimate();

  error_[0] = m1[0] - measurement_[0];
  error_[1] = m1[1] - measurement_[1];
  error_[2] = m2[0] - measurement_[2];
  error_[3] = m2[1] - measurement_[3];
}

void EdgeSE2TwoPointsXY::initialEstimate(
    const OptimizableGraph::VertexSet& fixed,
    OptimizableGraph::Vertex* toEstimate) {
  (void)toEstimate;

  assert(initialEstimatePossible(fixed, toEstimate) &&
         "Bad vertices specified");

  auto* pose = vertexXnRaw<0>();
  auto* v1 = vertexXnRaw<1>();
  auto* v2 = vertexXnRaw<2>();

  bool estimatev1 = true;
  bool estimatev2 = true;

  for (const auto& it : fixed) {
    if (v1->id() == it->id()) {
      estimatev1 = false;
    } else if (v2->id() == it->id()) {
      estimatev2 = false;
    }
  }

  if (estimatev1) {
    Vector2 submeas(measurement_[0], measurement_[1]);
    v1->setEstimate(pose->estimate() * submeas);
  }

  if (estimatev2) {
    Vector2 submeas(measurement_[2], measurement_[3]);
    v2->setEstimate(pose->estimate() * submeas);
  }
}

double EdgeSE2TwoPointsXY::initialEstimatePossible(
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

bool EdgeSE2TwoPointsXY::setMeasurementFromState() {
  auto* pose = vertexXnRaw<0>();
  auto* xy1 = vertexXnRaw<1>();
  auto* xy2 = vertexXnRaw<2>();

  Vector2 m1 = pose->estimate().inverse() * xy1->estimate();
  Vector2 m2 = pose->estimate().inverse() * xy2->estimate();

  measurement_[0] = m1[0];
  measurement_[1] = m1[1];
  measurement_[2] = m2[0];
  measurement_[3] = m2[1];
  return true;
}

}  // end namespace g2o
