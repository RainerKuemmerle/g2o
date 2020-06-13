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

TEST(IoSixDofExpmap, ReadWriteEdgeProjectXYZ2UVU) {
  readWriteVectorBasedEdge<EdgeProjectXYZ2UVU>();
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
