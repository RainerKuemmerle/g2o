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

#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sclam2d/edge_se2_sensor_calib.h"
#include "unit_test/test_helper/allocate_optimizer.h"

using SensorCalibrationParam = std::tuple<int, g2o::SE2>;

/**
 * Test fixture for performing the tests for estimating the sensor offset.
 */
class SensorCalibration
    : public ::testing::TestWithParam<SensorCalibrationParam> {
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
    auto computeMeasurement = [&sensorOffset](const g2o::SE2& v1,
                                              const g2o::SE2& v2) {
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
                          const std::shared_ptr<g2o::VertexSE2>& to,
                          const g2o::SE2& measurement) {
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
      const auto t =
          randomizePose(computeMeasurement(prev->estimate(), cur->estimate()));
      optimizer->addEdge(createEdge(prev, cur, t));
    }

    // generate loop closure edges
    for (int f = 1; f < numLaps; ++f) {
      for (int nn = 0; nn < nodesPerLoop; ++nn) {
        const auto& from = vertices[(f - 1) * nodesPerLoop + nn];
        for (int n = -1; n <= 1; ++n) {
          if (f == numLaps - 1 && n == 1) continue;
          const auto& to = vertices[f * nodesPerLoop + nn + n];
          const auto t = randomizePose(
              computeMeasurement(from->estimate(), to->estimate()));
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

// real world data problem
static std::string problem() {
  std::stringstream result;
  result << "VERTEX_SE2 1 0 0 0" << std::endl;
  result << "FIX 1" << std::endl;
  result << "VERTEX_SE2 2 0 0 0" << std::endl;
  result << "VERTEX_SE2 3 0.9802 0.00128 0.07651" << std::endl;
  result << "VERTEX_SE2 4 1.94214 0.04612 0.06036" << std::endl;
  result << "VERTEX_SE2 5 2.89714 0.15208 -0.05234" << std::endl;
  result << "VERTEX_SE2 6 3.83821 0.11379 -0.0258" << std::endl;
  result << "VERTEX_SE2 7 4.84846 0.07349 -0.02184" << std::endl;
  result << "VERTEX_SE2 8 5.90449 0.00628 -0.04111" << std::endl;
  result << "VERTEX_SE2 9 6.97665 -0.05104 0.08347" << std::endl;
  result << "VERTEX_SE2 10 7.93156 0.03124 0.08839" << std::endl;
  result << "VERTEX_SE2 11 8.85674 0.10356 0.07637" << std::endl;
  result << "VERTEX_SE2 12 9.78573 0.19927 -3.03362" << std::endl;
  result << "VERTEX_SE2 13 8.81109 0.10037 -2.94964" << std::endl;
  result << "VERTEX_SE2 14 7.85955 -0.0802 -2.90932" << std::endl;
  result << "VERTEX_SE2 15 6.89482 -0.35877 -2.98672" << std::endl;
  result << "VERTEX_SE2 16 5.87089 -0.51041 -3.01956" << std::endl;
  result << "VERTEX_SE2 17 4.8941 -0.66012 -3.04686" << std::endl;
  result << "VERTEX_SE2 18 3.86703 -0.81211 -3.11581" << std::endl;
  result << "VERTEX_SE2 19 2.84514 -0.86475 -3.09083" << std::endl;
  result << "VERTEX_SE2 20 1.8694 -0.91707 -3.06156" << std::endl;
  result << "VERTEX_SE2 21 0.89101 -1.04016 -2.99387" << std::endl;
  result << "VERTEX_SE2 22 -0.08696 -1.22328 0.14418" << std::endl;
  result << "EDGE_SE2_CALIB 1 3 2 0.9802 0.00128 0.07651 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 3 4 2 0.96255 -0.02883 -0.01616 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 4 5 2 0.95966 0.04817 -0.11269 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 5 6 2 0.94178 0.01099 0.02653 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 6 7 2 1.01095 -0.01422 0.00396 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 7 8 2 1.05725 -0.04413 -0.01927 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 8 9 2 1.0736 -0.01321 0.12459 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 9 10 2 0.95845 0.00237 0.00492 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 10 11 2 0.92795 -0.00964 -0.01203 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 11 12 2 0.93359 0.02456 -3.10998 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 12 13 2 0.97962 -0.00671 0.08397 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 13 14 2 0.96851 -0.00428 0.04032 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 14 15 2 1.00296 0.04903 -0.0774 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 15 16 2 1.03506 -0.00813 -0.03285 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 16 17 2 0.98775 0.02971 -0.02729 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 17 18 2 1.03685 0.05414 -0.06896 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 18 19 2 1.0229 0.02628 0.02498 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 19 20 2 0.97714 0.00275 0.02927 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 20 21 2 0.98509 0.04448 0.06769 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 21 22 2 0.99427 0.03718 3.13805 800 0 0 600 0 400"
         << std::endl;
  result << "EDGE_SE2_CALIB 1 22 2 0.00706 0.04984 -0.02723 800 0 0 600 0 400"
         << std::endl;
  return result.str();
}

using SensorCalibrationFromDataParam = std::tuple<std::string, int, g2o::SE2>;

/**
 * Test fixture for performing the tests from data files
 */
class SensorCalibrationFromData
    : public ::testing::TestWithParam<SensorCalibrationFromDataParam> {
 protected:
  void SetUp() override {
    optimizer = g2o::internal::createLmOptimizerForTests();

    const std::string data = std::get<0>(GetParam());
    const int sensorOffsetId = std::get<1>(GetParam());
    expectedSensorOffset = std::get<2>(GetParam());

    std::stringstream input(data);
    optimizer->load(input);
    sensorOffsetVertex = std::dynamic_pointer_cast<g2o::VertexSE2>(
        optimizer->vertex(sensorOffsetId));
  }

 protected:
  std::unique_ptr<g2o::SparseOptimizer> optimizer;
  std::shared_ptr<g2o::VertexSE2> sensorOffsetVertex;
  g2o::SE2 expectedSensorOffset;
};

TEST_P(SensorCalibrationFromData, Optimization) {
  auto before = sensorOffsetVertex->estimate().toVector();
  optimizer->initializeOptimization();
  optimizer->optimize(10);
  auto after = sensorOffsetVertex->estimate().toVector();
  EXPECT_NE(before, after);
  EXPECT_TRUE(after.isApprox(expectedSensorOffset.toVector(), 1e-2));
}

INSTANTIATE_TEST_SUITE_P(Sclam, SensorCalibrationFromData,
                         testing::Values(std::make_tuple(
                             problem(), 2,
                             g2o::SE2(-0.0281218, -0.284067, 0.036455))));
