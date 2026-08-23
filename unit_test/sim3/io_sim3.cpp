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

#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"

using namespace std;
using namespace g2o;

struct RandomSim3 {
  static Sim3 create() {
    Vector3 randomPosition = Vector3::Random();
    Quaternion randomOrientation(Vector4::Random().normalized());
    return Sim3(randomOrientation, randomPosition, 1.0);
  }
  static bool isApprox(const Sim3& a, const Sim3& b) {
    return a.translation().isApprox(b.translation(), 1e-5) &&
           a.rotation().isApprox(b.rotation(), 1e-5) &&
           fabs(a.scale() - b.scale()) < 1e-5;
  }
};

TEST(IoSim3, ReadWriteVertexSim3Expmap) {
  readWriteVectorBasedVertex<VertexSim3Expmap, RandomSim3>();
}

TEST(IoSim3, ReadWriteEdgeSim3) {
  readWriteVectorBasedEdge<EdgeSim3, RandomSim3>();
}

TEST(IoSim3, ReadWriteEdgeSim3ProjectXYZ) {
  readWriteVectorBasedEdge<EdgeSim3ProjectXYZ>();
}

TEST(IoSim3, ReadWriteEdgeInverseSim3ProjectXYZ) {
  readWriteVectorBasedEdge<EdgeInverseSim3ProjectXYZ>();
}

/*
 * Optional trailing fields
 *
 * focal_length2, principle_point2 and the fix_scale flag were added to the
 * record after the fact; vertices written before that must still read.
 */
namespace {

// Serialize v and drop the last dropTokens tokens, emulating a record
// written by a version of g2o that did not know about those fields.
std::string recordWithoutLastTokens(const VertexSim3Expmap& v, int dropTokens) {
  std::stringstream full;
  v.write(full);

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

void fillVertex(VertexSim3Expmap& v) {
  v.setEstimate(RandomSim3::create());
  v._focal_length1 = Vector2(100., 200.);
  v._principle_point1 = Vector2(300., 400.);
  v._focal_length2 = Vector2(500., 600.);
  v._principle_point2 = Vector2(700., 800.);
  v._fix_scale = true;
}

}  // namespace

TEST(IoSim3, VertexSim3ExpmapRoundTripsOptionalFields) {
  VertexSim3Expmap out;
  fillVertex(out);

  std::stringstream data;
  ASSERT_TRUE(out.write(data));

  VertexSim3Expmap in;
  ASSERT_TRUE(in.read(data));
  EXPECT_TRUE(out._focal_length2.isApprox(in._focal_length2, 1e-9));
  EXPECT_TRUE(out._principle_point2.isApprox(in._principle_point2, 1e-9));
  EXPECT_TRUE(in._fix_scale);
}

TEST(IoSim3, VertexSim3ExpmapReadsRecordWithoutOptionalFields) {
  VertexSim3Expmap out;
  fillVertex(out);
  // Drop focal_length2 (2), principle_point2 (2) and fix_scale (1).
  std::stringstream data(recordWithoutLastTokens(out, 5));

  VertexSim3Expmap in;
  in._focal_length2 = Vector2(11., 12.);
  in._principle_point2 = Vector2(13., 14.);
  in._fix_scale = true;
  ASSERT_TRUE(in.read(data)) << "record without optional fields was rejected";
  EXPECT_TRUE(out._focal_length1.isApprox(in._focal_length1, 1e-9));
  EXPECT_TRUE(out._principle_point1.isApprox(in._principle_point1, 1e-9));
  EXPECT_TRUE(Vector2(11., 12.).isApprox(in._focal_length2, 1e-9));
  EXPECT_TRUE(Vector2(13., 14.).isApprox(in._principle_point2, 1e-9));
  EXPECT_TRUE(in._fix_scale) << "absent fix_scale flag must be left alone";
}

TEST(IoSim3, VertexSim3ExpmapRejectsTruncatedOptionalFields) {
  VertexSim3Expmap out;
  fillVertex(out);
  // focal_length2 present, principle_point2 and fix_scale cut short.
  std::stringstream data(recordWithoutLastTokens(out, 3));

  VertexSim3Expmap in;
  EXPECT_FALSE(in.read(data))
      << "read() accepted a record whose optional fields were cut short";
}
