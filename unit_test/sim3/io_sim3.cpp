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
    return a.translation().isApprox(b.translation(), 1e-5) && a.rotation().isApprox(b.rotation(), 1e-5) &&
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
