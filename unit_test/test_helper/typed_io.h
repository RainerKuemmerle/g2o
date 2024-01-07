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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <string>

#include "g2o/core/io/io_format.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/string_tools.h"
#include "unit_test/test_helper/allocate_optimizer.h"
#include "unit_test/test_helper/random_state.h"

template <typename T>
struct FixedSizeEdgeIO : public ::testing::Test {
 public:
  FixedSizeEdgeIO() = default;

  void SetUp() override {
    using EdgeType = T;

    // Construct a small graph
    auto edge = std::make_shared<EdgeType>();

    // Initialize the vertices of the edge to a random state
    for (size_t i = 0; i < edge->vertices().size(); ++i) {
      auto v =
          std::shared_ptr<g2o::OptimizableGraph::Vertex>(edge->createVertex(i));
      edge->vertices()[i] = v;
      v->setId(i);
      v->setFixed(i == 0);
      initializeNthVertex<EdgeType::kNrOfVertices - 1>(i, *edge);
      this->optimizer_ptr_->addVertex(v);
    }

    // Generate a random information matrix
    const typename EdgeType::InformationType information_matrix = []() {
      const typename EdgeType::InformationType aux =
          EdgeType::InformationType::Random();
      typename EdgeType::InformationType result = 0.5 * (aux + aux.transpose());
      result.diagonal().array() += result.cols();
      return result;
    }();

    edge->setInformation(information_matrix);
    edge->setMeasurement(
        g2o::internal::RandomValue<typename EdgeType::Measurement>::create());
    this->optimizer_ptr_->addEdge(edge);
  }

 protected:
  std::unique_ptr<g2o::SparseOptimizer> optimizer_ptr_ =
      g2o::internal::createOptimizerForTests();

  template <int I, typename EdgeType>
  static std::enable_if_t<I == -1, void> initializeNthVertex(int /*i*/,
                                                             EdgeType& /*t*/) {}

  template <int I, typename EdgeType>
  static std::enable_if_t<I != -1, void> initializeNthVertex(int i,
                                                             EdgeType& t) {
    if (i == I) {
      auto vertex = t.template vertexXn<I>();
      vertex->setEstimate(
          g2o::internal::RandomValue<typename EdgeType::template VertexXnType<
              I>::EstimateType>::create());
      return;
    }
    initializeNthVertex<I - 1, EdgeType>(i, t);
  }
};
TYPED_TEST_SUITE_P(FixedSizeEdgeIO);

TYPED_TEST_P(FixedSizeEdgeIO, SaveAndLoad) {
  using namespace testing;  // NOLINT
  constexpr g2o::io::Format kFormat = g2o::io::Format::kG2O;

  std::stringstream buffer;
  bool save_result = this->optimizer_ptr_->save(buffer, kFormat);
  ASSERT_THAT(save_result, IsTrue());
  EXPECT_THAT(buffer.str(), Not(IsEmpty()));

  auto loaded_optimizer = g2o::internal::createOptimizerForTests();
  loaded_optimizer->load(buffer, kFormat);

  // Check that serialization result is the same
  std::stringstream buffer_after_loading;
  save_result = loaded_optimizer->save(buffer_after_loading, kFormat);
  ASSERT_THAT(save_result, IsTrue());

  // Prepare check of serialization result
  const std::vector<std::string> before_lines =
      g2o::strSplit(buffer.str(), "\n");
  const std::vector<std::string> after_lines =
      g2o::strSplit(buffer_after_loading.str(), "\n");
  ASSERT_THAT(before_lines, SizeIs(after_lines.size()));

  // Compare before and after line by line
  for (std::size_t i = 0; i < before_lines.size(); ++i) {
    const std::vector<std::string> before_tokens =
        g2o::strSplit(before_lines[i], " ");
    const std::vector<std::string> after_tokens =
        g2o::strSplit(after_lines[i], " ");
    ASSERT_THAT(before_tokens, SizeIs(after_tokens.size()));
    if (before_tokens.empty()) continue;
    EXPECT_THAT(before_tokens.front(), Eq(after_tokens.front()));
    for (std::size_t j = 1; j < before_tokens.size(); ++j) {
      const std::string& before_token = before_tokens[j];
      const std::string& after_token = after_tokens[j];
      const double before_value = std::stod(before_token);
      const double after_value = std::stod(after_token);
      EXPECT_THAT(before_value, DoubleNear(after_value, 1e-4))
          << " Line " << i << " differs token " << j << ": " << before_tokens[j]
          << " " << after_tokens[j];
    }
  }
}

REGISTER_TYPED_TEST_SUITE_P(FixedSizeEdgeIO, SaveAndLoad);

namespace g2o::internal {
class DefaultTypeNames {
 public:
  template <typename T>
  static std::string GetName(int) {
    return testing::internal::GetTypeName<T>();
  }
};

}  // namespace g2o::internal
