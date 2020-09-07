// g2o - General Graph Optimization
// Copyright (C) 2020 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include "g2o/types/slam3d_addons/edge_plane.h"
#include "g2o/types/slam3d_addons/edge_se3_calib.h"
#include "g2o/types/slam3d_addons/edge_se3_line.h"
#include "g2o/types/slam3d_addons/edge_se3_plane_calib.h"
#include "g2o/types/slam3d_addons/vertex_line3d.h"
#include "g2o/types/slam3d_addons/vertex_plane.h"
#include "g2o/types/slam3d_addons/vertex_se3_euler.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"
#include "unit_test/test_helper/random_state.h"

using namespace std;
using namespace g2o;

struct RandomPlane3D {
  static Plane3D create() { return Plane3D(Vector4::Random()); }
  static bool isApprox(const Plane3D& a, const Plane3D& b) { return a.coeffs().isApprox(b.coeffs(), 1e-5); }
};

struct RandomLine3D {
  static Line3D create() { return Line3D(Vector6::Random()); }
  static bool isApprox(const Line3D& a, const Line3D& b) { return a.isApprox(b, 1e-5); }
};

class IoSlam3dAddonsParam : public ::testing::Test {
 protected:
  void SetUp() override {
    graph.reset(new g2o::OptimizableGraph);

    // setting up parameters for tests
    paramOffset = new ParameterSE3Offset();
    paramOffset->setId(63);
    graph->addParameter(paramOffset);

    // setting up some vertices
    int numVertices = 0;
    pose = new VertexSE3;
    pose->setId(numVertices++);
    graph->addVertex(pose);
    line = new VertexLine3D;
    line->setId(numVertices++);
    graph->addVertex(line);
  }

  std::shared_ptr<g2o::OptimizableGraph> graph;
  ParameterSE3Offset* paramOffset = nullptr;
  VertexSE3* pose = nullptr;
  VertexLine3D* line = nullptr;
};

TEST(IoSlam3dAddOns, ReadWriteVertexSE3Euler) {
  readWriteVectorBasedVertex<VertexSE3Euler, internal::RandomIsometry3>();
}

TEST(IoSlam3dAddOns, ReadWriteVertexPlane) {
  // setting up the vertex for output
  VertexPlane* outputVertex = new VertexPlane();
  outputVertex->color = Vector3::Random();
  // IO Test
  readWriteVectorBasedVertex<VertexPlane, RandomPlane3D>(outputVertex);
}

TEST(IoSlam3dAddOns, ReadWriteVertexLine3D) { readWriteVectorBasedVertex<VertexLine3D, RandomLine3D>(); }

TEST(IoSlam3dAddOns, ReadWriteEdgeSE3Calib) { readWriteVectorBasedEdge<EdgeSE3Calib, internal::RandomIsometry3>(); }

TEST_F(IoSlam3dAddonsParam, ReadWriteEdgeSE3Line3D) {
  // prepare edge
  EdgeSE3Line3D* outputEdge = new EdgeSE3Line3D();
  outputEdge->setParameterId(0, paramOffset->id());
  outputEdge->setVertex(0, pose);
  outputEdge->setVertex(1, line);
  ASSERT_TRUE(graph->addEdge(outputEdge));
  // test IO
  readWriteVectorBasedEdge<EdgeSE3Line3D, RandomLine3D>(outputEdge);
}

TEST(IoSlam3dAddOns, ReadWriteEdgeSE3PlaneSensorCalib) { readWriteVectorBasedEdge<EdgeSE3PlaneSensorCalib, RandomPlane3D>(); }
