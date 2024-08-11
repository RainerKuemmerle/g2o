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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>

#include "g2o/core/eigen_types.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "unit_test/test_helper/allocate_optimizer.h"
#include "unit_test/test_helper/eigen_matcher.h"

using namespace testing;  // NOLINT

G2O_USE_TYPE_GROUP(slam2d);  // NOLINT

namespace {
class MockVertexSE2 : public g2o::VertexSE2 {
 public:
  MOCK_METHOD(void, push, (), (override));
};

class MockAction : public g2o::HyperGraphAction {
 public:
  bool operator()(const g2o::HyperGraph& graph,
                  g2o::HyperGraphAction::Parameters& params) override {
    return BracketOperator(graph, params);
  };
  MOCK_METHOD(bool, BracketOperator,
              (const g2o::HyperGraph&, g2o::HyperGraphAction::Parameters&), ());
};
}  // namespace

TEST(OptimizationBasics, BackupLevenberg) {
  constexpr size_t kNumVertices = 10;
  constexpr int kNumIter = 1;
  std::shared_ptr<MockVertexSE2> gauge;
  std::shared_ptr<MockVertexSE2> free_vertex;
  std::vector<std::shared_ptr<MockVertexSE2>> mock_vertices;

  std::shared_ptr<g2o::SparseOptimizer> optimizer =
      g2o::internal::createLmOptimizerForTests();

  // Add vertices
  for (size_t i = 0; i < kNumVertices; ++i) {
    auto v = std::make_shared<MockVertexSE2>();
    v->setEstimate(g2o::SE2());
    v->setId(i);
    if (i == 0) {
      v->setFixed(true);  // fix the first vertex
      gauge = v;
    } else {
      free_vertex = v;
    }
    optimizer->addVertex(v);
    mock_vertices.emplace_back(std::move(v));
  }

  // Add edges
  for (size_t i = 0; i < kNumVertices; ++i) {
    auto e1 = std::make_shared<g2o::EdgeSE2>();
    e1->vertices()[0] = optimizer->vertex((i + 0) % kNumVertices);
    e1->vertices()[1] = optimizer->vertex((i + 1) % kNumVertices);
    e1->setMeasurement(g2o::SE2(1., 0., 0.));
    e1->setInformation(g2o::EdgeSE2::InformationType::Identity());
    optimizer->addEdge(e1);
  }

  // Mock calls setup
  for (auto& v : mock_vertices) {
    EXPECT_CALL(*v, push).Times(AtLeast(1)).WillRepeatedly([&v]() {
      (*v).g2o::VertexSE2::push();
    });
  }

  optimizer->initializeOptimization();
  EXPECT_THAT(optimizer->activeVertices(), Contains(free_vertex));
  EXPECT_THAT(optimizer->activeVertices(), Contains(gauge));
  EXPECT_THAT(optimizer->activeVertices(), SizeIs(kNumVertices));

  optimizer->optimize(kNumIter);

  EXPECT_THAT(free_vertex->stackSize(), Eq(0));
}

TEST(OptimizationBasics, ActionCalls) {
  constexpr size_t kNumVertices = 10;
  constexpr int kNumIter = 1;

  std::shared_ptr<g2o::SparseOptimizer> optimizer =
      g2o::internal::createOptimizerForTests();

  // Add vertices
  for (size_t i = 0; i < kNumVertices; ++i) {
    auto v = std::make_shared<MockVertexSE2>();
    v->setEstimate(g2o::SE2(g2o::Vector3::Random()));
    v->setId(i);
    v->setFixed(i == 0);  // fix the first vertex
    optimizer->addVertex(v);
  }

  // Add edges
  for (size_t i = 0; i < kNumVertices; ++i) {
    auto e1 = std::make_shared<g2o::EdgeSE2>();
    e1->vertices()[0] = optimizer->vertex((i + 0) % kNumVertices);
    e1->vertices()[1] = optimizer->vertex((i + 1) % kNumVertices);
    e1->setMeasurement(g2o::SE2(g2o::Vector3::Random()));
    e1->setInformation(g2o::EdgeSE2::InformationType::Identity());
    optimizer->addEdge(e1);
  }

  // Mock Setup
  auto generic_action = std::make_shared<MockAction>();
  auto pre_action = std::make_shared<MockAction>();
  auto post_action = std::make_shared<MockAction>();
  Sequence action_sequence;

  ON_CALL(*generic_action, BracketOperator).WillByDefault(Return(true));
  ON_CALL(*pre_action, BracketOperator).WillByDefault(Return(true));
  ON_CALL(*post_action, BracketOperator).WillByDefault(Return(true));
  EXPECT_CALL(*generic_action, BracketOperator).Times(AtLeast(kNumIter * 2));
  // pre iteration
  EXPECT_CALL(*pre_action, BracketOperator).InSequence(action_sequence);
  EXPECT_CALL(*post_action, BracketOperator).InSequence(action_sequence);
  // actual iterations
  for (int i = 0; i < kNumIter; ++i) {
    EXPECT_CALL(*pre_action, BracketOperator).InSequence(action_sequence);
    EXPECT_CALL(*post_action, BracketOperator).InSequence(action_sequence);
  }

  optimizer->addPreIterationAction(generic_action);
  optimizer->addPreIterationAction(pre_action);
  optimizer->addPostIterationAction(generic_action);
  optimizer->addPostIterationAction(post_action);
  optimizer->initializeOptimization();
  optimizer->optimize(kNumIter);
}
