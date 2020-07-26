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

#include "g2o/types/sclam2d/edge_se2_odom_differential_calib.h"
#include "g2o/types/sclam2d/edge_se2_sensor_calib.h"
#include "g2o/types/sclam2d/vertex_odom_differential_params.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"

using namespace std;
using namespace g2o;

namespace {
struct RandomSE2 {
  static SE2 create() {
    auto randomPosition = Vector2::Random();
    auto randomOrientation = Vector2::Random();
    return SE2(randomPosition.x(), randomPosition.y(), std::atan2(randomOrientation.y(), randomOrientation.x()));
  }
  static bool isApprox(const SE2& a, const SE2& b) { return a.toVector().isApprox(b.toVector(), 1e-5); }
};

struct RandomVelocityMeasurement {
  static VelocityMeasurement create() {
    auto randomValues = Vector3::Random();
    return VelocityMeasurement(randomValues(0), randomValues(1), randomValues(2));
  }
  static bool isApprox(const VelocityMeasurement& a, const VelocityMeasurement& b) {
    return a.measurement().isApprox(b.measurement(), 1e-5) && fabs(a.dt() - b.dt()) < 1e-5;
  }
};
}  // namespace

TEST(IoSclam2d, ReadWriteVertexOdomDifferentialParams) { readWriteVectorBasedVertex<VertexOdomDifferentialParams>(); }

TEST(IoSclam2d, ReadWriteEdgeSE2SensorCalib) { readWriteVectorBasedEdge<EdgeSE2SensorCalib, RandomSE2>(); }

TEST(IoSclam2d, ReadWriteEdgeSE2OdomDifferentialCalib) {
  readWriteVectorBasedEdge<EdgeSE2OdomDifferentialCalib, RandomVelocityMeasurement>();
}
