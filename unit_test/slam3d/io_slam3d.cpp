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
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"
#include "unit_test/test_helper/random_state.h"

using namespace std;
using namespace g2o;

static std::shared_ptr<g2o::OptimizableGraph> createGraphWithPoseOffsetParam()
{
  std::shared_ptr<g2o::OptimizableGraph> graph(new g2o::OptimizableGraph);

  ParameterSE3Offset* paramOffset = new ParameterSE3Offset();
  paramOffset->setId(42);
  graph->addParameter(paramOffset);

  return graph;
}
/*
 * VERTEX Tests
 */
TEST(IoSlam3d, ReadWriteVertexSE3) {
  readWriteVectorBasedVertex<VertexSE3, internal::RandomIsometry3>();
}

TEST(IoSlam3d, ReadWriteVertexPointXYZ) {
  readWriteVectorBasedVertex<VertexPointXYZ>();
}

/*
 * EDGE Tests
 */
TEST(IoSlam3d, ReadWriteEdgeSE3) {
  readWriteVectorBasedEdge<EdgeSE3, internal::RandomIsometry3>();
}

TEST(IoSlam3d, ReadWriteEdgePointXYZ) {
  readWriteVectorBasedEdge<EdgePointXYZ>();
}

TEST(IoSlam3d, ReadWriteEdgeSE3Offset) {
  // set up an empty graph for adding the edge
  auto graph = createGraphWithPoseOffsetParam();

  ParameterSE3Offset* paramOffset2 = new ParameterSE3Offset();
  paramOffset2->setId(1337);
  graph->addParameter(paramOffset2);

  // setting up some vertices
  VertexSE3* p1 = new VertexSE3;
  p1->setId(0);
  graph->addVertex(p1);
  VertexSE3* p2 = new VertexSE3;
  p2->setId(1);
  graph->addVertex(p2);

  // setting up the edge for output
  EdgeSE3Offset* outputEdge = new EdgeSE3Offset();
  outputEdge->setParameterId(0, 42);
  outputEdge->setParameterId(1, paramOffset2->id());
  outputEdge->setVertex(0, p1);
  outputEdge->setVertex(1, p2);
  graph->addEdge(outputEdge);

  readWriteVectorBasedEdge<EdgeSE3Offset, internal::RandomIsometry3>(outputEdge);
}

TEST(IoSlam3d, ReadWriteEdgeSE3PointXYZ) {
  // set up an empty graph for adding the edge
  auto graph = createGraphWithPoseOffsetParam();

  // setting up some vertices
  VertexSE3* pose = new VertexSE3;
  pose->setId(0);
  graph->addVertex(pose);
  VertexPointXYZ* point = new VertexPointXYZ;
  point->setId(1);
  graph->addVertex(point);

  // setting up the edge for output
  EdgeSE3PointXYZ* outputEdge = new EdgeSE3PointXYZ();
  outputEdge->setParameterId(0, 42);
  outputEdge->setVertex(0, pose);
  outputEdge->setVertex(1, point);
  graph->addEdge(outputEdge);

  readWriteVectorBasedEdge<EdgeSE3PointXYZ>(outputEdge);
}

TEST(IoSlam3d, ReadWriteEdgeSE3Prior) {
  // set up an empty graph for adding the edge
  auto graph = createGraphWithPoseOffsetParam();

  // setting up some vertices
  VertexSE3* pose = new VertexSE3;
  pose->setId(0);
  graph->addVertex(pose);

    // setting up the edge for output
  EdgeSE3Prior* outputEdge = new EdgeSE3Prior();
  outputEdge->setParameterId(0, 42);
  outputEdge->setVertex(0, pose);
  graph->addEdge(outputEdge);

  readWriteVectorBasedEdge<EdgeSE3Prior, internal::RandomIsometry3>(outputEdge);
}
