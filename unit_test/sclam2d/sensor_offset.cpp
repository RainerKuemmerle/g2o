// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include <gtest/gtest.h>

#include <cmath>
#include <tuple>

#include "allocate_optimizer.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sclam2d/edge_se2_sensor_calib.h"

using SensorCalibrationParam = std::tuple<int, g2o::SE2>;

/**
 * Test fixture for performing the tests for estimating the sensor offset.
 */
class SensorCalibration : public ::testing::TestWithParam<SensorCalibrationParam> {
 protected:
  void SetUp() override {
    optimizer = g2o::internal::createLmOptimizerForTests();

    const int nodesPerLoop = std::get<0>(GetParam());
    const g2o::SE2 sensorOffset = std::get<1>(GetParam());
    constexpr double radius = 1.;
    constexpr int numLaps = 4;

    // helper functions
    auto randomizePose = [](const g2o::SE2& p) {
      g2o::Vector3 asVec = p.toVector();
      asVec(0) += g2o::Sampler::gaussRand(0., 0.025);
      asVec(1) += g2o::Sampler::gaussRand(0., 0.025);
      asVec(2) += g2o::Sampler::gaussRand(0., g2o::deg2rad(2.5));
      return g2o::SE2(asVec);
    };
    auto computeMeasurement = [&sensorOffset](const g2o::SE2& v1, const g2o::SE2& v2) {
      const g2o::SE2 transformation = v1.inverse() * v2;
      return sensorOffset.inverse() * transformation * sensorOffset;
    };

    const g2o::SE2 randomizedSensorOffset = randomizePose(sensorOffset);

    // sensor offset
    sensorOffsetVertex = std::make_shared<g2o::VertexSE2>();
    sensorOffsetVertex->setId(1 << 20);
    sensorOffsetVertex->setEstimate(randomizedSensorOffset);
    optimizer->addVertex(sensorOffsetVertex);

    // create the vertices
    int id = 0;
    for (int f = 0; f < numLaps; ++f) {
      for (int n = 0; n < nodesPerLoop; ++n) {
        const g2o::SE2 vertexPose = [&]() {
          const auto sign = f % 2 == 0 ? 1. : -1.;
          const g2o::Rotation2D rot(sign * 2. * n * M_PI / nodesPerLoop);
          const g2o::Vector2 trans = rot * g2o::Vector2(radius, 0.);
          return g2o::SE2(trans(0), trans(1), rot.angle());
        }();

        auto v = std::make_shared<g2o::VertexSE2>();
        v->setFixed(id == 0);
        v->setId(id++);
        v->setEstimate(vertexPose);
        optimizer->addVertex(v);
        vertices.emplace_back(v);
      }
    }

    // helper to create a sensor calibration edge
    auto createEdge = [&](const std::shared_ptr<g2o::VertexSE2>& from,
                          const std::shared_ptr<g2o::VertexSE2>& to, const g2o::SE2& measurement) {
      const g2o::Matrix3 information = g2o::Vector3(1., 1., 5.).asDiagonal();
      auto e = std::make_shared<g2o::EdgeSE2SensorCalib>();
      e->setVertex(0, from);
      e->setVertex(1, to);
      e->setVertex(2, sensorOffsetVertex);
      e->setMeasurement(measurement);
      e->setInformation(information);
      return e;
    };

    // generate odometry edges
    for (size_t i = 1; i < vertices.size(); ++i) {
      const auto& prev = vertices[i - 1];
      const auto& cur = vertices[i];
      const auto t = randomizePose(computeMeasurement(prev->estimate(), cur->estimate()));
      optimizer->addEdge(createEdge(prev, cur, t));
    }

    // generate loop closure edges
    for (int f = 1; f < numLaps; ++f) {
      for (int nn = 0; nn < nodesPerLoop; ++nn) {
        const auto& from = vertices[(f - 1) * nodesPerLoop + nn];
        for (int n = -1; n <= 1; ++n) {
          if (f == numLaps - 1 && n == 1) continue;
          const auto& to = vertices[f * nodesPerLoop + nn + n];
          const auto t = randomizePose(computeMeasurement(from->estimate(), to->estimate()));
          optimizer->addEdge(createEdge(from, to, t));
        }
      }
    }

    // apply noise
  }

 protected:
  std::unique_ptr<g2o::SparseOptimizer> optimizer;
  std::vector<std::shared_ptr<g2o::VertexSE2>> vertices;
  std::shared_ptr<g2o::VertexSE2> sensorOffsetVertex;
};

TEST_P(SensorCalibration, Optimization) {
  auto before = sensorOffsetVertex->estimate().toVector();
  optimizer->initializeOptimization();
  optimizer->optimize(10);
  auto after = sensorOffsetVertex->estimate().toVector();
  EXPECT_NE(before, after);
}

INSTANTIATE_TEST_SUITE_P(Sclam, SensorCalibration,
                         testing::Values(
                             // clang-format off
                             // number of poses, sensor offset
                             std::make_tuple(4, g2o::SE2()),
                             std::make_tuple(10, g2o::SE2(0.3, 0.2, 0.1)),
                             std::make_tuple(100, g2o::SE2(0.3, 0.2, 0.1))
                             // clang-format on
                             ));
