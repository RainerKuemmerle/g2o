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
#include "unit_test/test_helper/random_state.h"

using namespace std;
using namespace g2o;

struct RandomSBACam {
  static SBACam create() { return SBACam(); } // TODO Randomize
  static bool isApprox(const SBACam& a, const SBACam& b) {
    return a.toVector().isApprox(b.toVector(), 1e-5) && a.Kcam.isApprox(b.Kcam, 1e-5);
  }
};

/*
 * VERTEX Tests
 */
TEST(IoSba, ReadWriteVertexIntrinsics) {
  readWriteVectorBasedVertex<VertexIntrinsics>();
}

TEST(IoSba, ReadWriteVertexCam) {
  readWriteVectorBasedVertex<VertexCam, RandomSBACam>();
}

TEST(IoSba, ReadWriteVertexSBAPointXYZ) {
  readWriteVectorBasedVertex<VertexSBAPointXYZ>();
}

/*
 * EDGE Tests
 */
TEST(IoSba, ReadWriteEdgeProjectP2MC) {
  readWriteVectorBasedEdge<EdgeProjectP2MC>();
}

TEST(IoSba, ReadWriteEdgeProjectP2SC) {
  readWriteVectorBasedEdge<EdgeProjectP2SC>();
}

TEST(IoSba, ReadWriteEdgeSBACam) {
  readWriteVectorBasedEdge<EdgeSBACam, RandomSBACam>();
}

TEST(IoSba, ReadWriteEdgeSBAScale) {
  readWriteVectorBasedEdge<EdgeSBAScale, internal::RandomDouble>();
}
