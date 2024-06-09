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

#include <algorithm>
#include <unordered_set>

#include "g2o/simulator/simulator3d_base.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_depth.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_disparity.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "gmock/gmock.h"
#include "unit_test/test_helper/graph_functions.h"
using namespace g2o;      // NOLINT
using namespace testing;  // NOLINT

TEST(Simulator3D, Empty) {
  Simulator3D simulator;
  simulator.setup();
  simulator.simulate();

  const auto& graph = simulator.world().graph();

  EXPECT_THAT(graph.vertices(), SizeIs(0));
  EXPECT_THAT(graph.edges(), IsEmpty());
  EXPECT_THAT(graph.parameters(), SizeIs(0));
}

TEST(Simulator3D, Odom) {
  Simulator3D simulator;
  simulator.config.hasOdom = true;
  simulator.config.nlandmarks = 10;

  simulator.setup();
  simulator.simulate();

  const auto& graph = simulator.world().graph();

  EXPECT_THAT(simulator.graph().vertices(),
              SizeIs(simulator.config.simSteps + 1));
  // count pose edges
  const int odom_cnt =
      std::count_if(graph.edges().begin(), graph.edges().end(),
                    [](const OptimizableGraph::EdgeSet::key_type& edge) {
                      auto* odom = dynamic_cast<EdgeSE3*>(edge.get());
                      return odom;
                    });
  EXPECT_THAT(simulator.config.simSteps, Eq(odom_cnt));
}

TEST(Simulator3D, NoLandmarks) {
  Simulator3D simulator;
  simulator.config.hasPointSensor = true;

  simulator.setup();
  simulator.simulate();

  const OptimizableGraph& graph = simulator.world().graph();

  EXPECT_THAT(graph.vertices(), SizeIs(0));
  EXPECT_THAT(graph.edges(), IsEmpty());
  EXPECT_THAT(graph.parameters(), SizeIs(0));
}

class Simulator3DTests : public ::testing::TestWithParam<Simulator3D::Config> {
 protected:
  void SetUp() override {
    simulator_.config = GetParam();
    simulator_.config.nlandmarks = std::max(100, simulator_.config.nlandmarks);
  }

  Simulator3D simulator_;
};

TEST_P(Simulator3DTests, Simulate) {
  simulator_.setup();
  simulator_.simulate();

  const OptimizableGraph& graph = simulator_.world().graph();
  const Simulator3D::Config& config = simulator_.config;

  if (config.hasOdom || config.hasPoseSensor || config.hasPointSensor ||
      config.hasCompass || config.hasGPS) {
    EXPECT_THAT(g2o::internal::countVerticesMatchingType<VertexSE3>(graph),
                Gt(0));
  }

  if (config.hasPointSensor || config.hasPointDepthSensor ||
      config.hasPointDisparitySensor) {
    EXPECT_THAT(g2o::internal::countVerticesMatchingType<VertexPointXYZ>(graph),
                Gt(0));
  }

  // Base configuration
  if (config.hasOdom || config.hasPoseSensor) {
    EXPECT_THAT(g2o::internal::countEdgesMatchingType<EdgeSE3>(graph), Gt(0));
  }
  if (config.hasPointSensor) {
    EXPECT_THAT(g2o::internal::countEdgesMatchingType<EdgeSE3PointXYZ>(graph),
                Gt(0));
  }

  /* TODO(Rainer): Add simulation of a compass
    if (config.hasCompass)
      EXPECT_THAT(countEdgesMatchingType<EdgeSE2PointXY>(graph), Gt(0));
  */
  if (config.hasGPS) {
    EXPECT_THAT(g2o::internal::countEdgesMatchingType<EdgeSE3Prior>(graph),
                Gt(0));
  }

  // 2D specific configuration
  if (config.hasPointDisparitySensor) {
    EXPECT_THAT(
        g2o::internal::countEdgesMatchingType<EdgeSE3PointXYZDisparity>(graph),
        Gt(0));
  }
  if (config.hasPointDepthSensor) {
    EXPECT_THAT(
        g2o::internal::countEdgesMatchingType<EdgeSE3PointXYZDepth>(graph),
        Gt(0));
  }
}

namespace {
Simulator3D::Config FromWords(const std::unordered_set<std::string>& words) {
  Simulator3D::Config result;
  result.hasOdom = words.count("hasOdom") != 0;
  result.hasPoseSensor = words.count("hasPoseSensor") != 0;
  result.hasPointSensor = words.count("hasPointSensor") != 0;
  result.hasCompass = words.count("hasCompass") != 0;
  result.hasGPS = words.count("hasGPS") != 0;
  result.hasPointDisparitySensor = words.count("hasPointDisparitySensor") != 0;
  result.hasPointDepthSensor = words.count("hasPointDepthSensor") != 0;
  return result;
}

std::vector<Simulator3D::Config> ConfigsToTest() {
  std::vector<Simulator3D::Config> result;
  // single sensors
  result.push_back(FromWords({"hasOdom"}));
  result.push_back(FromWords({"hasPoseSensor"}));
  result.push_back(FromWords({"hasGPS"}));
  result.push_back(FromWords({"hasPointSensor"}));
  result.push_back(FromWords({"hasPointDisparitySensor"}));
  result.push_back(FromWords({"hasPointDepthSensor"}));
  // multiple
  result.push_back(FromWords({"hasOdom", "hasPoseSensor"}));
  result.push_back(FromWords({"hasOdom", "hasPoseSensor", "hasGPS"}));
  result.push_back(FromWords({"hasOdom", "hasPointSensor"}));
  return result;
}
}  // namespace

INSTANTIATE_TEST_SUITE_P(Simulator, Simulator3DTests,
                         ValuesIn(ConfigsToTest()));
