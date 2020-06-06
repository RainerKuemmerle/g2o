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

#include "g2o/types/slam3d/edge_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_offset.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"

using namespace std;
using namespace g2o;

/*
 * VERTEX Tests
*/
TEST(IoSlam3d, ReadWriteVertexSE3) {
  Isometry3 estimate = static_cast<Isometry3>(AngleAxis(1., Vector3(0.1, 0.2, 0.3).normalized()));
  estimate.translation() = Vector3(3, 2, 1);
  VertexSE3 outputVertex;
  outputVertex.setEstimate(estimate);
  VertexSE3 inputVertex;
  readWriteGraphElement(outputVertex, &inputVertex);
  ASSERT_TRUE(outputVertex.estimate().isApprox(inputVertex.estimate(), 1e-5));
}

TEST(IoSlam3d, ReadWriteVertexPointXYZ) {
  VertexPointXYZ outputVertex;
  outputVertex.setEstimate(Vector3::Random());
  VertexPointXYZ inputVertex;
  readWriteGraphElement(outputVertex, &inputVertex);
  ASSERT_TRUE(outputVertex.estimate().isApprox(inputVertex.estimate(), 1e-5));
}


/*
 * EDGE Tests
*/
TEST(IoSlam3d, ReadWriteEdgeSE3) {
  Isometry3 measurement = static_cast<Isometry3>(AngleAxis(1., Vector3(0.2, 0.3, 0.4).normalized()));
  measurement.translation() = Vector3(1, 2, 3);
  EdgeSE3 outputEdge;
  outputEdge.setMeasurement(measurement);
  randomizeInformationMatrix(outputEdge.information());
  EdgeSE3 inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().isApprox(inputEdge.measurement(), 1e-5));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSlam3d, ReadWriteEdgePointXYZ) {
  EdgePointXYZ outputEdge;
  outputEdge.setMeasurement(Vector3::Random());
  randomizeInformationMatrix(outputEdge.information());
  EdgePointXYZ inputEdge;
  readWriteGraphElement(outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge.measurement().isApprox(inputEdge.measurement(), 1e-5));
  ASSERT_TRUE(outputEdge.information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSlam3d, ReadWriteEdgeSE3Offset) {
  // set up an empty graph for adding the edge and its param
  g2o::OptimizableGraph graph;

  ParameterSE3Offset* paramOffset1 = new ParameterSE3Offset();
  paramOffset1->setId(42);
  graph.addParameter(paramOffset1);

  ParameterSE3Offset* paramOffset2 = new ParameterSE3Offset();
  paramOffset2->setId(1337);
  graph.addParameter(paramOffset2);

  // setting up some vertices
  VertexSE3* p1 = new VertexSE3;
  p1->setId(0);
  graph.addVertex(p1);
  VertexSE3* p2 = new VertexSE3;
  p2->setId(1);
  graph.addVertex(p2);

  Isometry3 measurement = static_cast<Isometry3>(AngleAxis(1., Vector3::Random().normalized()));
  measurement.translation() = Vector3::Random();

  // setting up the edge for output
  EdgeSE3Offset* outputEdge = new EdgeSE3Offset();
  outputEdge->setMeasurement(measurement);
  outputEdge->setParameterId(0, paramOffset1->id());
  outputEdge->setParameterId(1, paramOffset1->id());
  outputEdge->setVertex(0, p1);
  outputEdge->setVertex(1, p2);
  graph.addEdge(outputEdge);

  randomizeInformationMatrix(outputEdge->information());
  EdgeSE3Offset inputEdge;
  readWriteGraphElement(*outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge->measurement().isApprox(inputEdge.measurement(), 1e-5));
  ASSERT_TRUE(outputEdge->information().isApprox(inputEdge.information(), 1e-5));
}

TEST(IoSlam3d, ReadWriteEdgeSE3PointXYZ) {
  // set up an empty graph for adding the edge and its param
  g2o::OptimizableGraph graph;

  ParameterSE3Offset* paramOffset = new ParameterSE3Offset();
  paramOffset->setId(42);
  graph.addParameter(paramOffset);

  // setting up some vertices
  VertexSE3* pose = new VertexSE3;
  pose->setId(0);
  graph.addVertex(pose);
  VertexPointXYZ* point = new VertexPointXYZ;
  point->setId(1);
  graph.addVertex(point);

  // setting up the edge for output
  EdgeSE3PointXYZ* outputEdge = new EdgeSE3PointXYZ();
  outputEdge->setMeasurement(Vector3::Random());
  outputEdge->setParameterId(0, paramOffset->id());
  outputEdge->setVertex(0, pose);
  outputEdge->setVertex(1, point);
  graph.addEdge(outputEdge);

  randomizeInformationMatrix(outputEdge->information());
  EdgeSE3PointXYZ inputEdge;
  readWriteGraphElement(*outputEdge, &inputEdge);
  ASSERT_TRUE(outputEdge->measurement().isApprox(inputEdge.measurement(), 1e-5));
  ASSERT_TRUE(outputEdge->information().isApprox(inputEdge.information(), 1e-5));
}
