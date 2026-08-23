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
#include <string>
#include <vector>

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"
#include "unit_test/test_helper/random_state.h"

using namespace std;
using namespace g2o;

/*
 * VERTEX Tests
 */
TEST(IoSixDofExpmap, ReadWriteVertexSE3Expmap) {
  readWriteVectorBasedVertex<VertexSE3Expmap, internal::RandomSE3Quat>();
}

/*
 * EDGE Tests
 */
TEST(IoSixDofExpmap, ReadWriteEdgeSE3Expmap) {
  readWriteVectorBasedEdge<EdgeSE3Expmap, internal::RandomSE3Quat>();
}

TEST(IoSixDofExpmap, ReadWriteEdgeSE3ProjectXYZ) {
  readWriteVectorBasedEdge<EdgeSE3ProjectXYZ>();
}

TEST(IoSixDofExpmap, ReadWriteEdgeStereoSE3ProjectXYZ) {
  readWriteVectorBasedEdge<EdgeStereoSE3ProjectXYZ>();
}

TEST(IoSixDofExpmap, ReadWriteEdgeSE3ProjectXYZOnlyPose) {
  readWriteVectorBasedEdge<EdgeSE3ProjectXYZOnlyPose>();
}

TEST(IoSixDofExpmap, ReadWriteEdgeStereoSE3ProjectXYZOnlyPose) {
  readWriteVectorBasedEdge<EdgeStereoSE3ProjectXYZOnlyPose>();
}

/*
 * EDGE Tests including a Camera Parameter
 */
class IoSixDofExpmapParam : public ::testing::Test {
 protected:
  void SetUp() override {
    graph.reset(new g2o::OptimizableGraph);

    CameraParameters* paramCam = new CameraParameters();
    paramCam->setId(42);
    graph->addParameter(paramCam);

    // setting up some vertices
    point = new VertexPointXYZ;
    point->setId(0);
    graph->addVertex(point);
    pose = new VertexSE3Expmap;
    pose->setId(1);
    graph->addVertex(pose);
  }

  void prepareEdge(OptimizableGraph::Edge* e) {
    e->setParameterId(0, 42);
    e->setVertex(0, point);
    e->setVertex(1, pose);
    graph->addEdge(e);
  }

  std::shared_ptr<g2o::OptimizableGraph> graph;
  VertexPointXYZ* point = nullptr;
  VertexSE3Expmap* pose = nullptr;
};

TEST_F(IoSixDofExpmapParam, ReadWriteEdgeProjectXYZ2UV) {
  EdgeProjectXYZ2UV* outputEdge = new EdgeProjectXYZ2UV;
  prepareEdge(outputEdge);
  readWriteVectorBasedEdge<EdgeProjectXYZ2UV>(outputEdge);
}

TEST_F(IoSixDofExpmapParam, ReadWriteEdgeProjectPSI2UV) {
  VertexSE3Expmap* p2 = new VertexSE3Expmap;
  p2->setId(2);
  graph->addVertex(p2);
  EdgeProjectPSI2UV* outputEdge = new EdgeProjectPSI2UV;
  outputEdge->setVertex(2, p2);
  prepareEdge(outputEdge);
  readWriteVectorBasedEdge<EdgeProjectPSI2UV>(outputEdge);
}

TEST_F(IoSixDofExpmapParam, ReadWriteEdgeProjectXYZ2UVU) {
  EdgeProjectXYZ2UVU* outputEdge = new EdgeProjectXYZ2UVU;
  prepareEdge(outputEdge);
  readWriteVectorBasedEdge<EdgeProjectXYZ2UVU>(outputEdge);
}

/*
 * Optional trailing fields
 *
 * The camera intrinsics (and, for the OnlyPose edges, the world-frame
 * landmark) were added to the record after the fact. Files written before
 * that must still read, and a record that stops early must not swallow the
 * tokens of the following line.
 */
namespace {

// Serialize e and drop the last dropTokens tokens, emulating a record
// written by a version of g2o that did not know about those fields.
template <typename EdgeType>
std::string recordWithoutLastTokens(const EdgeType& e, int dropTokens) {
  std::stringstream full;
  e.write(full);

  std::istringstream iss(full.str());
  std::vector<std::string> tokens;
  for (std::string t; iss >> t;) tokens.push_back(t);

  EXPECT_GT(static_cast<int>(tokens.size()), dropTokens)
      << "test needs a record longer than the number of dropped tokens";

  std::string out;
  for (int i = 0; i + dropTokens < static_cast<int>(tokens.size()); ++i)
    out += tokens[i] + " ";
  return out;
}

void fillProjectXYZ(EdgeSE3ProjectXYZ& e) {
  e.setMeasurement(Vector2(1., 2.));
  e.setInformation(Matrix2::Identity() * 3.);
  e.fx = 100.;
  e.fy = 200.;
  e.cx = 300.;
  e.cy = 400.;
}

}  // namespace

TEST(IoSixDofExpmap, EdgeSE3ProjectXYZRoundTripsIntrinsics) {
  EdgeSE3ProjectXYZ out;
  fillProjectXYZ(out);

  std::stringstream data;
  ASSERT_TRUE(out.write(data));

  EdgeSE3ProjectXYZ in;
  ASSERT_TRUE(in.read(data));
  EXPECT_DOUBLE_EQ(out.fx, in.fx);
  EXPECT_DOUBLE_EQ(out.fy, in.fy);
  EXPECT_DOUBLE_EQ(out.cx, in.cx);
  EXPECT_DOUBLE_EQ(out.cy, in.cy);
}

TEST(IoSixDofExpmap, EdgeSE3ProjectXYZReadsRecordWithoutIntrinsics) {
  EdgeSE3ProjectXYZ out;
  fillProjectXYZ(out);
  std::stringstream data(recordWithoutLastTokens(out, 4));

  EdgeSE3ProjectXYZ in;
  in.fx = 7.;
  in.fy = 8.;
  in.cx = 9.;
  in.cy = 10.;
  ASSERT_TRUE(in.read(data)) << "record without intrinsics was rejected";
  EXPECT_TRUE(out.measurement().isApprox(in.measurement(), 1e-9));
  EXPECT_TRUE(out.information().isApprox(in.information(), 1e-9));
  EXPECT_DOUBLE_EQ(7., in.fx) << "absent intrinsics must be left alone";
  EXPECT_DOUBLE_EQ(8., in.fy);
  EXPECT_DOUBLE_EQ(9., in.cx);
  EXPECT_DOUBLE_EQ(10., in.cy);
}

TEST(IoSixDofExpmap, EdgeSE3ProjectXYZRejectsTruncatedIntrinsics) {
  EdgeSE3ProjectXYZ out;
  fillProjectXYZ(out);
  std::stringstream data(recordWithoutLastTokens(out, 2));

  EdgeSE3ProjectXYZ in;
  EXPECT_FALSE(in.read(data))
      << "read() accepted a record whose intrinsics were cut short";
}

TEST(IoSixDofExpmap, EdgeSE3ProjectXYZStopsAtEndOfLine) {
  EdgeSE3ProjectXYZ out;
  fillProjectXYZ(out);

  std::stringstream data;
  data << recordWithoutLastTokens(out, 4) << "\n"
       << "NEXT_RECORD 11 12 13 14\n";

  EdgeSE3ProjectXYZ in;
  in.fx = 7.;
  ASSERT_TRUE(in.read(data));
  EXPECT_DOUBLE_EQ(7., in.fx)
      << "read() took its intrinsics from the following line";

  std::string token;
  data >> token;
  EXPECT_EQ("NEXT_RECORD", token) << "the following line was consumed";
}

TEST(IoSixDofExpmap, EdgeStereoSE3ProjectXYZOnlyPoseRoundTripsOptionalFields) {
  EdgeStereoSE3ProjectXYZOnlyPose out;
  out.setMeasurement(Vector3(1., 2., 3.));
  out.setInformation(Matrix3::Identity() * 3.);
  out.Xw = Vector3(4., 5., 6.);
  out.fx = 100.;
  out.fy = 200.;
  out.cx = 300.;
  out.cy = 400.;
  out.bf = 500.;

  std::stringstream data;
  ASSERT_TRUE(out.write(data));

  EdgeStereoSE3ProjectXYZOnlyPose in;
  ASSERT_TRUE(in.read(data));
  EXPECT_TRUE(out.Xw.isApprox(in.Xw, 1e-9));
  EXPECT_DOUBLE_EQ(out.fx, in.fx);
  EXPECT_DOUBLE_EQ(out.bf, in.bf);

  // The same record without its optional tail leaves the edge untouched.
  std::stringstream legacy(recordWithoutLastTokens(out, 8));
  EdgeStereoSE3ProjectXYZOnlyPose legacyIn;
  legacyIn.Xw = Vector3(7., 8., 9.);
  legacyIn.bf = 11.;
  ASSERT_TRUE(legacyIn.read(legacy));
  EXPECT_TRUE(Vector3(7., 8., 9.).isApprox(legacyIn.Xw, 1e-9));
  EXPECT_DOUBLE_EQ(11., legacyIn.bf);
}
