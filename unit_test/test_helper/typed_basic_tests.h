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

#pragma once

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <string>
#include <tuple>

#include "g2o/core/io/io_format.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/string_tools.h"
#include "unit_test/test_helper/allocate_optimizer.h"
#include "unit_test/test_helper/eigen_matcher.h"
#include "unit_test/test_helper/random_state.h"
#include "unit_test/test_helper/utils.h"

namespace g2o::internal::testing {
/**
 * @brief Matcher for testing Numeric and Analytic Jacobian to be approximately
 * equal.
 */
MATCHER_P2(JacobianApproxEqual, expect, prec,
           std::string(negation ? "isn't" : "is") + " approx equal to" +
               ::testing::PrintToString(expect)) {
  if (arg.size() != expect.size()) return false;
  for (int j = 0; j < arg.size(); ++j) {
    double diff = std::abs(arg(j) - expect(j));
    if (diff > prec(arg(j), expect(j))) return false;
  }
  return true;
}
}  // namespace g2o::internal::testing

/**
 * @brief A typed test for Edges.
 *
 * @tparam T A tuple like type whereas the first type gives the type of the Edge
 * and the second determines the type of a parameter. Providing the type of a
 * parameter is optional.
 */
template <typename T>
struct FixedSizeEdgeBasicTests : public ::testing::Test {
 public:
  using EpsilonFunction = std::function<double(const double, const double)>;
  using EdgeType = typename std::tuple_element<0, T>::type;

  EpsilonFunction epsilon = [](const double, const double) { return 1e-3; };

  FixedSizeEdgeBasicTests() = default;

  void SetUp() override {
    constexpr std::size_t kTupleSize = std::tuple_size_v<T>;

    // Construct a small graph
    edge_ = std::make_shared<EdgeType>();

    // Initialize the vertices of the edge to a random state
    for (std::size_t i = 0; i < edge_->vertices().size(); ++i) {
      auto v = std::shared_ptr<g2o::OptimizableGraph::Vertex>(
          edge_->createVertex(i));
      edge_->vertices()[i] = v;
      v->setId(i);
      initializeNthVertex<EdgeType::kNrOfVertices - 1>(i, *edge_);
      this->optimizer_ptr_->addVertex(v);
    }

    // Add the parameter types if required
    if constexpr (kTupleSize > 1) {
      using ParamTuple = typename TupleTypes<T>::Tail;
      constexpr std::size_t kParamSize = std::tuple_size_v<ParamTuple>;
      for (std::size_t i = 0; i < kParamSize; ++i) {
        addNthParameter<kParamSize - 1, ParamTuple, EdgeType>(i, *edge_);
      }
    }

    // Generate a random information matrix
    const typename EdgeType::InformationType information_matrix = []() {
      const typename EdgeType::InformationType aux =
          EdgeType::InformationType::Random();
      typename EdgeType::InformationType result = 0.5 * (aux + aux.transpose());
      result.diagonal().array() += result.cols();
      return result;
    }();

    edge_->setInformation(information_matrix);
    edge_->setMeasurement(
        g2o::internal::RandomValue<typename EdgeType::Measurement>::create());
    this->optimizer_ptr_->addEdge(edge_);

    this->numeric_jacobian_workspace_.updateSize(*edge_);
    this->numeric_jacobian_workspace_.allocate();
  }

 protected:
  std::shared_ptr<EdgeType> edge_;

  std::unique_ptr<g2o::SparseOptimizer> optimizer_ptr_ =
      g2o::internal::createOptimizerForTests();

  template <int I, typename EdgeType>
  static std::enable_if_t<I == -1, void> initializeNthVertex(int, EdgeType&) {}

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

  template <int I, typename ParameterTuple, typename EdgeType>
  std::enable_if_t<I == -1, void> addNthParameter(int, EdgeType&) {}

  template <int I, typename ParameterTuple, typename EdgeType>
  std::enable_if_t<I != -1, void> addNthParameter(int i, EdgeType& edge) {
    if (i != I) {
      addNthParameter<I - 1, ParameterTuple, EdgeType>(i, edge);
    }
    using ParameterType = typename std::tuple_element<I, ParameterTuple>::type;
    auto param = std::make_shared<ParameterType>();
    param->setId(i + 100);
    // TODO(Rainer): Modify the parameter to a random value
    this->optimizer_ptr_->addParameter(param);
    edge.setParameterId(i, param->id());
  }

  template <class TupleType>
  struct TupleTypes;

  template <class HeadType, class... TailTypes>
  struct TupleTypes<std::tuple<HeadType, TailTypes...>> {
    using Head = HeadType;
    using Tail = std::tuple<TailTypes...>;
  };

  /**
   * @brief Helper template to compute the Jacobian numerically and by the
   * implementation of the Edge class.
   *
   * @tparam Ints Index sequence over the EdgeType::kNrOfVertices
   */
  template <std::size_t... Ints>
  void compute(EdgeType& edge, std::index_sequence<Ints...> /*unused*/) {
    // calling the analytic Jacobian but writing to the numeric workspace
    edge.template BaseFixedSizedEdge<
        EdgeType::kDimension, typename EdgeType::Measurement,
        typename EdgeType::template VertexXnType<Ints>...>::
        linearizeOplus(this->numeric_jacobian_workspace_);
    // copy result into analytic workspace
    this->jacobian_workspace_ = this->numeric_jacobian_workspace_;
    this->numeric_jacobian_workspace_.setZero();

    // compute the numeric Jacobian into the this->numeric_jacobian_workspace_
    // workspace as setup by the previous call
    edge.template BaseFixedSizedEdge<
        EdgeType::kDimension, typename EdgeType::Measurement,
        typename EdgeType::template VertexXnType<Ints>...>::linearizeOplus();
  }

  g2o::JacobianWorkspace jacobian_workspace_;
  g2o::JacobianWorkspace numeric_jacobian_workspace_;
};
TYPED_TEST_SUITE_P(FixedSizeEdgeBasicTests);

TYPED_TEST_P(FixedSizeEdgeBasicTests, SaveAndLoad) {
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

TYPED_TEST_P(FixedSizeEdgeBasicTests, Jacobian) {
  using EdgeType = typename std::tuple_element<0, TypeParam>::type;

  this->compute(*(this->edge_),
                std::make_index_sequence<EdgeType::kNrOfVertices>());

  // compare the two Jacobians
  for (std::size_t i = 0; i < this->edge_->vertices().size(); ++i) {
    int numElems = EdgeType::kDimension;
    auto* vertex = static_cast<g2o::OptimizableGraph::Vertex*>(
        this->edge_->vertex(i).get());
    numElems *= vertex->dimension();
    g2o::VectorX::ConstMapType n(
        this->numeric_jacobian_workspace_.workspaceForVertex(i), numElems);
    g2o::VectorX::ConstMapType a(
        this->jacobian_workspace_.workspaceForVertex(i), numElems);
    EXPECT_THAT(g2o::internal::print_wrap(a),
                g2o::internal::testing::JacobianApproxEqual(
                    g2o::internal::print_wrap(n), this->epsilon));
  }
}

REGISTER_TYPED_TEST_SUITE_P(FixedSizeEdgeBasicTests, SaveAndLoad, Jacobian);

namespace g2o::internal {
class DefaultTypeNames {
 public:
  template <typename T>
  static std::string GetName(int) {
    return ExtractTupleHead(::testing::internal::GetTypeName<T>());
  }
};

}  // namespace g2o::internal
