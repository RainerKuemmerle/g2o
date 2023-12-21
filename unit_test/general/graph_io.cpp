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

#include <sstream>

#include "g2o/core/abstract_graph.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

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

/**
 * @brief Test fixture for IO with the abstract graph.
 */
class AbstractGraphIO : public TestWithParam<g2o::AbstractGraph::Format> {
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
        std::vector<double>{1., 2., 3., 4., 5., 6.},
        std::vector<g2o::AbstractGraph::AbstractData>{
            {"VERTEX_TAG", "more fancy data"}});
    abstract_graph_.edges().emplace_back(
        "EDGE_SE2", std::vector<int>{2, 3}, std::vector<double>{0.4, 0.5, 0.6},
        std::vector<double>{1.1, 2.1, 3.1, 4.1, 5.1, 6.1});
  }
  g2o::AbstractGraph abstract_graph_;
};

TEST_P(AbstractGraphIO, SaveAndLoad) {
  g2o::AbstractGraph::Format format = GetParam();
  g2o::AbstractGraph load_save_graph(abstract_graph_);

  std::stringstream buffer;
  bool save_result = load_save_graph.save(buffer, format);
  ASSERT_THAT(save_result, IsTrue());

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
}

INSTANTIATE_TEST_SUITE_P(AbstractGraph, AbstractGraphIO,
                         Values(g2o::AbstractGraph::Format::kG2O));
