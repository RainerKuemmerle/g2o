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

#include <ios>
#include <memory>
#include <sstream>
#include <tuple>

#include "g2o/config.h"  // IWYU pragma: keep
#include "g2o/core/base_dynamic_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/factory.h"
#include "g2o/core/io/io_format.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/edge_se2_lotsofxy.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/allocate_optimizer.h"

using namespace testing;  // NOLINT

class DynamicVertex : public g2o::BaseDynamicVertex<g2o::VectorX> {
 public:
  DynamicVertex() = default;

  void oplusImpl(const g2o::VectorX::MapType& update) override {
    estimate_ += update;
  }

 protected:
  bool setDimensionImpl(int newDimension) override {
    estimate_.resize(newDimension);
    return true;
  }
};

class EdgeForDynamicVertex
    : public g2o::BaseUnaryEdge<2, g2o::Vector2, DynamicVertex> {
 public:
  void computeError() override { error_.setZero(); }
};

/**
 * @brief Test fixture for IO with the abstract graph.
 */
class OptimizableGraphDynamicEdgeIO
    : public TestWithParam<std::tuple<g2o::io::Format, bool>> {
 protected:
  void SetUp() override {
    // Register the specific types of the test once
    g2o::Factory* factory = g2o::Factory::instance();
    if (!factory->knowsTag("TEST_DYN_VERTEX")) {
      factory->registerType(
          "TEST_DYN_VERTEX",
          std::make_unique<g2o::HyperGraphElementCreator<DynamicVertex>>());
    }
    if (!factory->knowsTag("TEST_EDGE_FOR_DYN")) {
      factory->registerType(
          "TEST_EDGE_FOR_DYN",
          std::make_unique<
              g2o::HyperGraphElementCreator<EdgeForDynamicVertex>>());
    }

    optimizer_ptr_ = g2o::internal::createOptimizerForTests();

    // Add vertices
    auto pose = std::make_shared<g2o::VertexSE2>();
    pose->setFixed(true);
    pose->setId(0);
    optimizer_ptr_->addVertex(pose);

    auto point0 = std::make_shared<g2o::VertexPointXY>();
    point0->setId(1);
    point0->setEstimate(g2o::Vector2::Random());
    optimizer_ptr_->addVertex(point0);

    auto point1 = std::make_shared<g2o::VertexPointXY>();
    point1->setId(2);
    point1->setEstimate(g2o::Vector2::Random());
    optimizer_ptr_->addVertex(point1);

    auto point2 = std::make_shared<g2o::VertexPointXY>();
    point2->setId(3);
    point2->setEstimate(g2o::Vector2::Random());
    optimizer_ptr_->addVertex(point2);

    // Add dynamically sized vertex
    constexpr int kDynVertexSize = 5;
    auto dyn_vertex = std::make_shared<DynamicVertex>();
    dyn_vertex->setId(4);
    dyn_vertex->setDimension(kDynVertexSize);
    dyn_vertex->setEstimate(g2o::VectorX::Random(kDynVertexSize));
    optimizer_ptr_->addVertex(dyn_vertex);

    auto edge = std::make_shared<g2o::EdgeSE2LotsOfXY>();
    edge->resize(4);
    edge->vertices()[0] = pose;
    edge->vertices()[1] = point0;
    edge->vertices()[2] = point1;
    edge->vertices()[3] = point2;
    edge->setInformation(
        g2o::VectorX::Ones(edge->information().cols()).asDiagonal());
    edge->setMeasurementFromState();
    optimizer_ptr_->addEdge(edge);

    auto edge_for_dyn = std::make_shared<EdgeForDynamicVertex>();
    edge_for_dyn->setInformation(
        EdgeForDynamicVertex::InformationType::Identity() * 5.);
    edge_for_dyn->setVertex(0, dyn_vertex);
    optimizer_ptr_->addEdge(edge_for_dyn);
  }
  std::unique_ptr<g2o::SparseOptimizer> optimizer_ptr_;
};

TEST_P(OptimizableGraphDynamicEdgeIO, SaveAndLoad) {
  const g2o::io::Format format = std::get<0>(GetParam());
  const bool supports_dynamic_edge = std::get<1>(GetParam());

  std::stringstream buffer(format == g2o::io::Format::kBinary
                               ? std::ios_base::binary | std::ios_base::in |
                                     std::ios_base::out
                               : std::ios_base::in | std::ios_base::out);
  bool save_result = optimizer_ptr_->save(buffer, format);
  ASSERT_THAT(save_result, IsTrue());
  EXPECT_THAT(buffer.str(), Not(IsEmpty()));
  if (!supports_dynamic_edge) {
    SUCCEED();
    return;
  }

  auto loaded_optimizer = g2o::internal::createOptimizerForTests();
  loaded_optimizer->load(buffer, format);

  EXPECT_THAT(loaded_optimizer->vertices(),
              SizeIs(optimizer_ptr_->vertices().size()));

  EXPECT_THAT(loaded_optimizer->vertices(),
              UnorderedElementsAre(Key(0), Key(1), Key(2), Key(3), Key(4)));

  EXPECT_THAT(loaded_optimizer->edges(),
              SizeIs(optimizer_ptr_->edges().size()));

  // Brutally check that serialization result is the same
  std::stringstream buffer_after_loading;
  save_result = loaded_optimizer->save(buffer_after_loading, format);
  ASSERT_THAT(save_result, IsTrue());
  EXPECT_THAT(buffer.str(), Eq(buffer_after_loading.str()));
}

namespace {
// We can always test G2O format, others depend on libraries
const auto kFileformatsToTest =
    Values(std::make_tuple(g2o::io::Format::kG2O, true)
#ifdef G2O_HAVE_JSON
               ,
           std::make_tuple(g2o::io::Format::kJson, true),
           std::make_tuple(g2o::io::Format::kBinary, true)
#endif
    );
}  // namespace

INSTANTIATE_TEST_SUITE_P(OptimizableGraphDynamic, OptimizableGraphDynamicEdgeIO,
                         kFileformatsToTest);
