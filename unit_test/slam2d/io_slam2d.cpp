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
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"

using namespace std;
using namespace g2o;

TEST(IoSlam2d, ReadWriteVertexSE2) {
  VertexSE2 outputVertex;
  outputVertex.setEstimate(SE2(1, 2, 0.3));
  VertexSE2 inputVertex;
  readWriteGraphElement(outputVertex, &inputVertex);
  ASSERT_TRUE(outputVertex.estimate().toVector().isApprox(inputVertex.estimate().toVector()));
}

TEST(IoSlam2d, ReadWriteVertexPointXY) {
  VertexPointXY outputVertex;
  outputVertex.setEstimate(Vector2(1, 2));
  VertexPointXY inputVertex;
  readWriteGraphElement(outputVertex, &inputVertex);
  ASSERT_TRUE(outputVertex.estimate().isApprox(inputVertex.estimate()));
}

TEST(IoSlam2d, ReadWriteEdgeSE2) {
  EdgeSE2 outputEdge;
  outputEdge.setMeasurement(SE2(2, 1, 0.5));
  randomizeInformationMatrix(outputEdge.information());
  EdgeSE2 inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().toVector().isApprox(inputEdge.measurement().toVector()));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSlam2d, ReadWriteEdgeSE2Prior) {
  EdgeSE2Prior outputEdge;
  outputEdge.setMeasurement(SE2(1, 2, -0.5));
  randomizeInformationMatrix(outputEdge.information());
  EdgeSE2Prior inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().toVector().isApprox(inputEdge.measurement().toVector()));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSlam2d, ReadWriteEdgeSE2PointXY) {
  EdgeSE2PointXY outputEdge;
  outputEdge.setMeasurement(Vector2(2., 1.));
  randomizeInformationMatrix(outputEdge.information());
  EdgeSE2PointXY inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().isApprox(inputEdge.measurement()));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSlam2d, ReadWriteEdgeXYPrior) {
  EdgeXYPrior outputEdge;
  outputEdge.setMeasurement(Vector2(3., 2.));
  randomizeInformationMatrix(outputEdge.information());
  EdgeXYPrior inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().isApprox(inputEdge.measurement()));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}
