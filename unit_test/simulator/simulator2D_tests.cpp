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
#include <string>
#include <unordered_set>

#include "g2o/core/optimizable_graph.h"
#include "g2o/simulator/simulator.h"
#include "g2o/simulator/simulator2d_base.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2_pointxy_bearing.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d_addons/edge_se2_segment2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace g2o;      // NOLINT
using namespace testing;  // NOLINT

namespace {
template <typename PoseType>
int countVerticesMatchingType(const OptimizableGraph& graph) {
  return std::count_if(
      graph.vertices().begin(), graph.vertices().end(),
      [](const OptimizableGraph::VertexIDMap::value_type& elem) {
        auto* ptr = dynamic_cast<PoseType*>(elem.second.get());
        return ptr != nullptr;
      });
}

template <typename EdgeType>
int countEdgesMatchingType(const OptimizableGraph& graph) {
  return std::count_if(graph.edges().begin(), graph.edges().end(),
                       [](const OptimizableGraph::EdgeSet::key_type& edge) {
                         auto* ptr = dynamic_cast<EdgeType*>(edge.get());
                         return ptr != nullptr;
                       });
}
}  // namespace

TEST(Simulator2D, Empty) {
  Simulator2D simulator;
  simulator.setup();
  simulator.simulate();

  const auto& graph = simulator.world().graph();

  EXPECT_THAT(graph.vertices(), SizeIs(0));
  EXPECT_THAT(graph.edges(), IsEmpty());
  EXPECT_THAT(graph.parameters(), SizeIs(0));
}

TEST(Simulator2D, Odom) {
  Simulator2D simulator;
  simulator.config.hasOdom = true;

  simulator.setup();
  simulator.simulate();

  const OptimizableGraph& graph = simulator.world().graph();

  EXPECT_THAT(graph.vertices(), SizeIs(simulator.config.simSteps + 1));
  const int poses = countVerticesMatchingType<VertexSE2>(graph);
  EXPECT_THAT(poses, Eq(simulator.config.simSteps + 1));
  const int odom_cnt = countEdgesMatchingType<EdgeSE2>(graph);
  EXPECT_THAT(simulator.config.simSteps, Eq(odom_cnt));
}

TEST(Simulator2D, NoLandmarks) {
  Simulator2D simulator;
  simulator.config.hasPointSensor = true;

  simulator.setup();
  simulator.simulate();

  const OptimizableGraph& graph = simulator.world().graph();

  EXPECT_THAT(graph.vertices(), SizeIs(0));
  EXPECT_THAT(graph.edges(), IsEmpty());
  EXPECT_THAT(graph.parameters(), SizeIs(0));
}

class Simulator2DTests : public ::testing::TestWithParam<Simulator2D::Config> {
 protected:
  void SetUp() override {
    simulator_.config = GetParam();
    simulator_.config.nlandmarks = std::max(100, simulator_.config.nlandmarks);
    simulator_.config.nSegments = std::max(100, simulator_.config.nSegments);
  }

  Simulator2D simulator_;
};

TEST_P(Simulator2DTests, Simulate) {
  simulator_.setup();
  simulator_.simulate();

  const OptimizableGraph& graph = simulator_.world().graph();
  const Simulator2D::Config& config = simulator_.config;

  if (config.hasOdom || config.hasPoseSensor || config.hasPointSensor ||
      config.hasPointBearingSensor || config.hasCompass || config.hasGPS)
    EXPECT_THAT(countVerticesMatchingType<VertexSE2>(graph), Gt(0));

  if (config.hasPointSensor || config.hasPointBearingSensor)
    EXPECT_THAT(countVerticesMatchingType<VertexPointXY>(graph), Gt(0));

  // Base configuration
  if (config.hasOdom || config.hasPoseSensor)
    EXPECT_THAT(countEdgesMatchingType<EdgeSE2>(graph), Gt(0));
  if (config.hasPointSensor)
    EXPECT_THAT(countEdgesMatchingType<EdgeSE2PointXY>(graph), Gt(0));

  /* TODO(Rainer): Add simulation of a compass
    if (config.hasCompass)
      EXPECT_THAT(countEdgesMatchingType<EdgeSE2PointXY>(graph), Gt(0));
  */
  if (config.hasGPS)
    EXPECT_THAT(countEdgesMatchingType<EdgeSE2Prior>(graph), Gt(0));

  // 2D specific configuration
  if (config.hasPointBearingSensor)
    EXPECT_THAT(countEdgesMatchingType<EdgeSE2PointXYBearing>(graph), Gt(0));
  if (config.hasSegmentSensor)
    EXPECT_THAT(countEdgesMatchingType<EdgeSE2Segment2D>(graph), Gt(0));
}

namespace {
Simulator2D::Config FromWords(const std::unordered_set<std::string>& words) {
  Simulator2D::Config result;
  result.hasOdom = words.count("hasOdom") != 0;
  result.hasPoseSensor = words.count("hasPoseSensor") != 0;
  result.hasPointSensor = words.count("hasPointSensor") != 0;
  result.hasCompass = words.count("hasCompass") != 0;
  result.hasGPS = words.count("hasGPS") != 0;
  result.hasPointBearingSensor = words.count("hasPointBearingSensor") != 0;
  result.hasSegmentSensor = words.count("hasSegmentSensor") != 0;
  return result;
}

std::vector<Simulator2D::Config> ConfigsToTest() {
  std::vector<Simulator2D::Config> result;
  // single sensors
  result.push_back(FromWords({"hasOdom"}));
  result.push_back(FromWords({"hasPoseSensor"}));
  result.push_back(FromWords({"hasGPS"}));
  result.push_back(FromWords({"hasPointSensor"}));
  result.push_back(FromWords({"hasPointBearingSensor"}));
  result.push_back(FromWords({"hasSegmentSensor"}));
  // multiple
  result.push_back(FromWords({"hasOdom", "hasPoseSensor"}));
  result.push_back(FromWords({"hasOdom", "hasPoseSensor", "hasGps"}));
  result.push_back(FromWords({"hasOdom", "hasPointSensor"}));
  return result;
}
}  // namespace

INSTANTIATE_TEST_SUITE_P(Simulator, Simulator2DTests,
                         ValuesIn(ConfigsToTest()));
