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

#include <algorithm>
#include <ios>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "g2o/config.h"  // IWYU pragma: keep
#include "g2o/core/abstract_graph.h"
#include "g2o/core/io/io_format.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/data/vertex_tag.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/allocate_optimizer.h"

using namespace testing;  // NOLINT

MATCHER(ParamEqual, "") {
  const auto& that = std::get<0>(arg);
  const auto& expected = std::get<1>(arg);
  return ExplainMatchResult(
      AllOf(Field("tag", &g2o::AbstractGraph::AbstractParameter::tag,
                  Eq(expected.tag)),
            Field("id", &g2o::AbstractGraph::AbstractParameter::id,
                  Eq(expected.id)),
            Field("value", &g2o::AbstractGraph::AbstractParameter::value,
                  ElementsAreArray(expected.value))),
      that, result_listener);
}

MATCHER(VertexEqual, "") {
  const auto& that = std::get<0>(arg);
  const auto& expected = std::get<1>(arg);
  // TODO(Rainer): compare data vector
  return ExplainMatchResult(
      AllOf(
          Field("tag", &g2o::AbstractGraph::AbstractVertex::tag,
                Eq(expected.tag)),
          Field("id", &g2o::AbstractGraph::AbstractVertex::id, Eq(expected.id)),
          Field("estimate", &g2o::AbstractGraph::AbstractVertex::estimate,
                ElementsAreArray(expected.estimate))),
      that, result_listener);
}

MATCHER(EdgeEqual, "") {
  const auto& that = std::get<0>(arg);
  const auto& expected = std::get<1>(arg);
  // TODO(Rainer): compare data vector
  return ExplainMatchResult(
      AllOf(Field("tag", &g2o::AbstractGraph::AbstractEdge::tag,
                  Eq(expected.tag)),
            Field("ids", &g2o::AbstractGraph::AbstractEdge::ids,
                  ElementsAreArray(expected.ids)),
            Field("param_ids", &g2o::AbstractGraph::AbstractEdge::param_ids,
                  ElementsAreArray(expected.param_ids)),
            Field("measurement", &g2o::AbstractGraph::AbstractEdge::measurement,
                  ElementsAreArray(expected.measurement)),
            Field("information", &g2o::AbstractGraph::AbstractEdge::information,
                  ElementsAreArray(expected.information))),
      that, result_listener);
}

namespace {
auto KeyMatch(const std::vector<int>& keys) {
  std::vector<decltype(Key(keys[0]))> matchers;
  matchers.reserve(keys.size());
  for (const auto& val : keys) matchers.push_back(Key(val));
  return UnorderedElementsAreArray(matchers);
};
}  // namespace

/**
 * @brief Test fixture for IO with the abstract graph.
 */
class AbstractGraphIO : public TestWithParam<g2o::io::Format> {
 protected:
  void SetUp() override {
    abstract_graph_.fixed() = {1};

    abstract_graph_.parameters().emplace_back("PARAMS_SE2OFFSET", 42,
                                              std::vector<double>{12., 13.});

    abstract_graph_.vertices().emplace_back("VERTEX_SE2", 1,
                                            std::vector<double>{1., 2., 3.});
    abstract_graph_.vertices().emplace_back("VERTEX_SE2", 2,
                                            std::vector<double>{2., 3., 4.});
    abstract_graph_.vertices().emplace_back(
        "VERTEX_SE2", 3, std::vector<double>{5., 6., 7.},
        std::vector<g2o::AbstractGraph::AbstractData>{
            {"VERTEX_TAG", "fancy data"}});

    abstract_graph_.edges().emplace_back(
        "EDGE_SE2", std::vector<int>{1, 2}, std::vector<double>{0.1, 0.2, 0.3},
        std::vector<double>{1., 2., 3., 4., 5., 6.}, std::vector<int>(),
        std::vector<g2o::AbstractGraph::AbstractData>{
            {"VERTEX_TAG", "more fancy data"}});
    abstract_graph_.edges().emplace_back(
        "EDGE_SE2", std::vector<int>{2, 3}, std::vector<double>{0.4, 0.5, 0.6},
        std::vector<double>{1.1, 2.1, 3.1, 4.1, 5.1, 6.1});
  }
  g2o::AbstractGraph abstract_graph_;
};

TEST_F(AbstractGraphIO, RenameTags) {
  EXPECT_THAT(abstract_graph_.vertices(),
              Contains(testing::Field(&g2o::AbstractGraph::AbstractVertex::tag,
                                      Eq("VERTEX_SE2")))
                  .Times(Ge(1)));

  auto v3_it = std::find_if(
      abstract_graph_.vertices().begin(), abstract_graph_.vertices().end(),
      [](const g2o::AbstractGraph::AbstractVertex& v) { return v.id == 3; });
  EXPECT_THAT(v3_it->data,
              Contains(testing::Field(&g2o::AbstractGraph::AbstractData::tag,
                                      Eq("VERTEX_TAG")))
                  .Times(Ge(1)));

  std::unordered_map<std::string, std::string> tag_map = {
      {"VERTEX_SE2", "VERTEX_BAR"}, {"VERTEX_TAG", "TAG_FOO"}};
  abstract_graph_.renameTags(tag_map);

  EXPECT_THAT(abstract_graph_.vertices(),
              Contains(testing::Field(&g2o::AbstractGraph::AbstractVertex::tag,
                                      Eq("VERTEX_SE2")))
                  .Times(0));
  EXPECT_THAT(abstract_graph_.vertices(),
              Contains(testing::Field(&g2o::AbstractGraph::AbstractVertex::tag,
                                      Eq("VERTEX_BAR")))
                  .Times(Ge(1)));
  EXPECT_THAT(v3_it->data,
              Contains(testing::Field(&g2o::AbstractGraph::AbstractData::tag,
                                      Eq("VERTEX_TAG")))
                  .Times(0));
  EXPECT_THAT(v3_it->data,
              Contains(testing::Field(&g2o::AbstractGraph::AbstractData::tag,
                                      Eq("TAG_FOO")))
                  .Times(Ge(1)));
}

TEST_P(AbstractGraphIO, SaveAndLoad) {
  g2o::io::Format format = GetParam();
  g2o::AbstractGraph load_save_graph(abstract_graph_);

  std::stringstream buffer(format == g2o::io::Format::kBinary
                               ? std::ios_base::binary | std::ios_base::in |
                                     std::ios_base::out
                               : std::ios_base::in | std::ios_base::out);
  bool save_result = load_save_graph.save(buffer, format);
  ASSERT_THAT(save_result, IsTrue());
  EXPECT_THAT(buffer.str(), Not(IsEmpty()));

  load_save_graph.clear();
  EXPECT_THAT(load_save_graph.fixed(), IsEmpty());
  EXPECT_THAT(load_save_graph.parameters(), IsEmpty());
  EXPECT_THAT(load_save_graph.vertices(), IsEmpty());
  EXPECT_THAT(load_save_graph.edges(), IsEmpty());

  bool load_status = load_save_graph.load(buffer, format);
  ASSERT_THAT(load_status, IsTrue());

  EXPECT_THAT(load_save_graph.fixed(),
              ElementsAreArray(abstract_graph_.fixed()));
  EXPECT_THAT(load_save_graph.parameters(),
              Pointwise(ParamEqual(), abstract_graph_.parameters()));
  EXPECT_THAT(load_save_graph.vertices(),
              Pointwise(VertexEqual(), abstract_graph_.vertices()));
  EXPECT_THAT(load_save_graph.edges(),
              Pointwise(EdgeEqual(), abstract_graph_.edges()));
}

/**
 * @brief Test fixture for IO with the abstract graph.
 */
class OptimizableGraphIO : public TestWithParam<g2o::io::Format> {
 protected:
  void SetUp() override {
    optimizer_ptr_ = g2o::internal::createOptimizerForTests();

    // Add vertices
    auto v0 = std::make_shared<g2o::VertexSE2>();
    v0->setFixed(true);
    v0->setId(0);
    optimizer_ptr_->addVertex(v0);

    auto data = std::make_shared<g2o::VertexTag>();
    data->setName("vertex_foobar");
    data->setHostname("my_robot");
    v0->setUserData(data);

    auto v1 = std::make_shared<g2o::VertexSE2>();
    v1->setId(1);
    optimizer_ptr_->addVertex(v1);

    auto v2 = std::make_shared<g2o::VertexSE2>();
    v2->setId(2);
    optimizer_ptr_->addVertex(v2);

    // Add edges
    auto e1 = std::make_shared<g2o::EdgeSE2>();
    e1->vertices()[0] = optimizer_ptr_->vertex(0);
    e1->vertices()[1] = optimizer_ptr_->vertex(1);
    e1->setMeasurement(g2o::SE2(1, 0, 0));
    e1->setInformation(g2o::MatrixN<3>::Identity());
    optimizer_ptr_->addEdge(e1);

    auto edge_data = std::make_shared<g2o::VertexTag>();
    edge_data->setName("foobar");
    edge_data->setHostname("my_robot");
    e1->setUserData(edge_data);

    auto e2 = std::make_shared<g2o::EdgeSE2>();
    e2->vertices()[0] = optimizer_ptr_->vertex(1);
    e2->vertices()[1] = optimizer_ptr_->vertex(2);
    e2->setMeasurement(g2o::SE2(0, 1, 0));
    e2->setInformation(g2o::MatrixN<3>::Identity());
    optimizer_ptr_->addEdge(e2);
  }
  std::unique_ptr<g2o::SparseOptimizer> optimizer_ptr_;
};

TEST_P(OptimizableGraphIO, SaveAndLoad) {
  g2o::io::Format format = GetParam();

  std::stringstream buffer(format == g2o::io::Format::kBinary
                               ? std::ios_base::binary | std::ios_base::in |
                                     std::ios_base::out
                               : std::ios_base::in | std::ios_base::out);
  bool save_result = optimizer_ptr_->save(buffer, format);
  ASSERT_THAT(save_result, IsTrue());
  EXPECT_THAT(buffer.str(), Not(IsEmpty()));

  auto loaded_optimizer = g2o::internal::createOptimizerForTests();
  loaded_optimizer->load(buffer, format);

  EXPECT_THAT(loaded_optimizer->vertices(),
              SizeIs(optimizer_ptr_->vertices().size()));

  const std::vector<int> expected_keys = [this]() {
    std::vector<int> result;
    for (const auto& p : optimizer_ptr_->vertices()) result.push_back(p.first);
    return result;
  }();
  EXPECT_THAT(loaded_optimizer->vertices(), KeyMatch(expected_keys));

  const std::vector<int> loaded_fixed = [&loaded_optimizer]() {
    std::vector<int> result;
    for (const auto& p : loaded_optimizer->vertices())
      if (static_cast<g2o::OptimizableGraph::Vertex*>(p.second.get())->fixed())
        result.push_back(p.first);
    return result;
  }();
  const std::vector<int> expected_fixed = [this]() {
    std::vector<int> result;
    for (const auto& p : optimizer_ptr_->vertices())
      if (static_cast<g2o::OptimizableGraph::Vertex*>(p.second.get())->fixed())
        result.push_back(p.first);
    return result;
  }();
  EXPECT_THAT(loaded_fixed, UnorderedElementsAreArray(expected_fixed));

  EXPECT_THAT(loaded_optimizer->edges(),
              SizeIs(optimizer_ptr_->edges().size()));
  // TODO(Rainer): Compare estimates of vertices
  // TODO(Rainer): Compare measurement of edges
  // TODO(Rainer): Compare information of edges

  // Brutally check that serialization result is the same
  std::stringstream buffer_after_loading;
  save_result = loaded_optimizer->save(buffer_after_loading, format);
  ASSERT_THAT(save_result, IsTrue());
  EXPECT_THAT(buffer.str(), Eq(buffer_after_loading.str()));
}

namespace {
// We can always test G2O format, others depend on libraries
const auto kFileformatsToTest = Values(
    g2o::io::Format::kG2O
#ifdef G2O_HAVE_CEREAL
    ,
    g2o::io::Format::kJson, g2o::io::Format::kXML, g2o::io::Format::kBinary
#endif
);
}  // namespace

INSTANTIATE_TEST_SUITE_P(AbstractGraph, AbstractGraphIO, kFileformatsToTest);
INSTANTIATE_TEST_SUITE_P(OptimizableGraphGraph, OptimizableGraphIO,
                         kFileformatsToTest);

TEST(OptimizableGraphIO, FileFilter) {
  EXPECT_THAT(g2o::io::getFileFilter(false), Not(IsEmpty()));
  EXPECT_THAT(g2o::io::getFileFilter(true), Not(IsEmpty()));
  EXPECT_THAT(g2o::io::getFileFilter(false),
              IsSubsetOf(g2o::io::getFileFilter(true)));
}

TEST(OptimizableGraphIO, FormatToString) {
  static constexpr g2o::io::Format kAllFormats[] = {
      g2o::io::Format::kUndefined, g2o::io::Format::kG2O,
      g2o::io::Format::kBinary, g2o::io::Format::kJson, g2o::io::Format::kXML};
  for (const auto& f : kAllFormats)
    EXPECT_THAT(g2o::io::to_string(f), Not(IsEmpty()));

  EXPECT_THAT(g2o::io::to_string(static_cast<g2o::io::Format>(1 << 31)),
              IsEmpty());
}
