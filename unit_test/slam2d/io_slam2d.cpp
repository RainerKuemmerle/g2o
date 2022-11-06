// g2o - General Graph Optimization
// Copyright (C) 2014 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include <sstream>

#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/edge_xy_prior.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"

namespace {
struct RandomSE2 {
  static g2o::SE2 create() {
    g2o::Vector2 randomPosition = g2o::Vector2::Random();
    g2o::Vector2 randomOrientation = g2o::Vector2::Random();
    return g2o::SE2(randomPosition.x(), randomPosition.y(),
                    std::atan2(randomOrientation.y(), randomOrientation.x()));
  }
  static bool isApprox(const g2o::SE2& a, const g2o::SE2& b) {
    return a.toVector().isApprox(b.toVector(), 1e-5);
  }
};
}  // namespace

TEST(IoSlam2d, ReadWriteVertexSE2) {
  g2o::readWriteVectorBasedVertex<g2o::VertexSE2, RandomSE2>();
}

TEST(IoSlam2d, ReadWriteVertexPointXY) {
  g2o::readWriteVectorBasedVertex<g2o::VertexPointXY>();
}

TEST(IoSlam2d, ReadWriteParameterSE2Offset) {
  g2o::ParameterSE2Offset outputParam;
  outputParam.setOffset(g2o::SE2(1., 2., 0.3));
  g2o::ParameterSE2Offset inputParam;
  readWriteGraphElement(outputParam, &inputParam);
  ASSERT_TRUE(
      outputParam.offset().toVector().isApprox(inputParam.offset().toVector()));
}

TEST(IoSlam2d, ReadWriteEdgeSE2) {
  g2o::readWriteVectorBasedEdge<g2o::EdgeSE2, RandomSE2>();
}

TEST(IoSlam2d, ReadWriteEdgeSE2Prior) {
  g2o::readWriteVectorBasedEdge<g2o::EdgeSE2Prior, RandomSE2>();
}

TEST(IoSlam2d, ReadWriteEdgeSE2PointXY) {
  g2o::readWriteVectorBasedEdge<g2o::EdgeSE2PointXY>();
}

TEST(IoSlam2d, ReadWriteEdgeXYPrior) {
  g2o::readWriteVectorBasedEdge<g2o::EdgeXYPrior>();
}
