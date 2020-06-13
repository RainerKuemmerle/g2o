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

#include "g2o/types/sba/types_sba.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"

using namespace std;
using namespace g2o;

/*
 * VERTEX Tests
 */
TEST(IoSba, ReadWriteVertexIntrinsics) {
  VertexIntrinsics outputVertex;
  outputVertex.setEstimate(VertexIntrinsics::EstimateType::Random());
  VertexIntrinsics inputVertex;
  readWriteGraphElement(outputVertex, &inputVertex);
  ASSERT_TRUE(outputVertex.estimate().isApprox(inputVertex.estimate(), 1e-5));
}

TEST(IoSba, ReadWriteVertexCam) {
  VertexCam outputVertex;
  VertexCam inputVertex;
  readWriteGraphElement(outputVertex, &inputVertex);
  ASSERT_TRUE(outputVertex.estimate().toVector().isApprox(inputVertex.estimate().toVector(), 1e-5));
  ASSERT_TRUE(outputVertex.estimate().Kcam.isApprox(inputVertex.estimate().Kcam, 1e-5));
  ASSERT_DOUBLE_EQ(outputVertex.estimate().baseline, inputVertex.estimate().baseline);
}

TEST(IoSba, ReadWriteVertexSBAPointXYZ) {
  VertexSBAPointXYZ outputVertex;
  outputVertex.setEstimate(Vector3::Random());
  VertexSBAPointXYZ inputVertex;
  readWriteGraphElement(outputVertex, &inputVertex);
  ASSERT_TRUE(outputVertex.estimate().isApprox(inputVertex.estimate(), 1e-5));
}

/*
 * EDGE Tests
 */
TEST(IoSba, ReadWriteEdgeProjectP2MC) {
  EdgeProjectP2MC outputEdge;
  outputEdge.setMeasurement(Vector2::Random());
  randomizeInformationMatrix(outputEdge.information());
  EdgeProjectP2MC inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().isApprox(inputEdge.measurement(), 1e-5));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSba, ReadWriteEdgeProjectP2SC) {
  EdgeProjectP2SC outputEdge;
  outputEdge.setMeasurement(Vector3::Random());
  randomizeInformationMatrix(outputEdge.information());
  EdgeProjectP2SC inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().isApprox(inputEdge.measurement(), 1e-5));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSba, ReadWriteEdgeSBACam) {
  EdgeSBACam outputEdge;
  outputEdge.setMeasurement(SE3Quat(Quaternion::UnitRandom(), Vector3::Random()));
  randomizeInformationMatrix(outputEdge.information());
  EdgeSBACam inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().toVector().isApprox(inputEdge.measurement().toVector(), 1e-5));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSba, ReadWriteEdgeSBAScale) {
  EdgeSBAScale outputEdge;
  outputEdge.setMeasurement(1.1);
  randomizeInformationMatrix(outputEdge.information());
  EdgeSBAScale inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_DOUBLE_EQ(outputEdge.measurement(), inputEdge.measurement());
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}
