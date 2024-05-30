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

#include <algorithm>
#include <cstddef>
#include <memory>
#include <numeric>
#include <unordered_set>
#include <utility>
#include <vector>

#include "g2o/core/eigen_types.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "unit_test/test_helper/allocate_optimizer.h"
#include "unit_test/test_helper/eigen_matcher.h"

using g2o::internal::print_wrap;
using namespace testing;  // NOLINT

G2O_USE_TYPE_GROUP(slam2d);  // NOLINT

namespace {
class JacobianWorkspaceTestAdapter : public g2o::JacobianWorkspace {
 public:
  [[nodiscard]] WorkspaceVector& workspace() { return workspace_; }
  [[nodiscard]] int maxNumVertices() const { return maxNumVertices_; }
  [[nodiscard]] int maxDimension() const { return maxDimension_; }
};
}  // namespace

TEST(General, BinaryEdgeConstructor) {
  g2o::EdgeSE2 e2;
  ASSERT_EQ(nullptr, e2.vertices()[0]);
  ASSERT_EQ(nullptr, e2.vertices()[1]);
}

TEST(General, GraphAddVertex) {
  auto optimizer = g2o::internal::createOptimizerForTests();

  auto v1 = std::make_shared<g2o::VertexSE2>();
  v1->setId(0);
  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_EQ(optimizer.get(), v1->graph());
  ASSERT_EQ(size_t(1), optimizer->vertices().size());
  ASSERT_FALSE(optimizer->addVertex(v1));
  ASSERT_EQ(size_t(1), optimizer->vertices().size());

  auto v1_from_optimizer = optimizer->vertex(0);
  ASSERT_EQ(v1.get(), v1_from_optimizer.get());

  {
    auto v2 = std::make_shared<g2o::VertexSE2>();
    v2->setId(0);
    ASSERT_FALSE(optimizer->addVertex(v2));
    ASSERT_EQ(size_t(1), optimizer->vertices().size());
    ASSERT_EQ(nullptr, v2->graph());
  }

  // removing vertex
  const bool removed = optimizer->removeVertex(v1, true);
  ASSERT_TRUE(removed);
  ASSERT_EQ(nullptr, v1->graph());
  ASSERT_THAT(optimizer->vertices(), testing::SizeIs(0));
}

TEST(General, GraphAddEdge) {
  auto optimizer = g2o::internal::createOptimizerForTests();

  auto v1 = std::make_shared<g2o::VertexSE2>();
  v1->setId(0);
  auto v2 = std::make_shared<g2o::VertexSE2>();
  v2->setId(1);

  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_TRUE(optimizer->addVertex(v2));

  auto e1 = std::make_shared<g2o::EdgeSE2>();
  e1->setVertex(0, v1);
  e1->setVertex(1, v2);
  ASSERT_TRUE(optimizer->addEdge(e1));
  ASSERT_EQ(optimizer.get(), e1->graph());
  ASSERT_FALSE(optimizer->addEdge(e1));
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().count(e1));
  ASSERT_EQ(size_t(1), v2->edges().count(e1));

  auto e2 = std::make_shared<g2o::EdgeSE2>();
  ASSERT_FALSE(optimizer->addEdge(e2))
      << "Adding edge with unset vertices was possible";
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(nullptr, e2->graph());

  auto e3 = std::make_shared<g2o::EdgeSE2>();
  e3->setVertex(0, v1);
  e3->setVertex(1, v1);
  ASSERT_FALSE(optimizer->addEdge(e3))
      << "Adding binary edge with same vertices was possible";
  ASSERT_EQ(size_t(1), optimizer->edges().size());

  const bool removed = optimizer->removeEdge(e1);
  ASSERT_TRUE(removed);
  ASSERT_TRUE(optimizer->edges().empty());
}

TEST(General, GraphAddVertexAndClear) {
  auto optimizer = g2o::internal::createOptimizerForTests();

  auto v1 = std::make_shared<g2o::VertexSE2>();
  v1->setId(0);
  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_THAT(optimizer->vertices(), testing::SizeIs(1));

  // clearing
  optimizer->clear();
  ASSERT_THAT(optimizer->vertices(), testing::SizeIs(0));
  ASSERT_EQ(nullptr, v1->graph());

  // re-add the same other optimizer again
  auto otherOptimizer = g2o::internal::createOptimizerForTests();
  ASSERT_TRUE(otherOptimizer->addVertex(v1));
  ASSERT_EQ(v1->graph(), otherOptimizer.get());
  ASSERT_THAT(otherOptimizer->vertices(), testing::SizeIs(1));
}

TEST(General, GraphAddGraph) {
  auto optimizer = g2o::internal::createOptimizerForTests();

  auto v1 = std::make_shared<g2o::VertexSE2>();
  v1->setId(0);
  auto v2 = std::make_shared<g2o::VertexSE2>();
  v2->setId(1);

  optimizer->addVertex(v1);
  optimizer->addVertex(v2);

  auto e1 = std::make_shared<g2o::EdgeSE2>();
  e1->setVertex(0, v1);
  e1->setVertex(1, v2);
  optimizer->addEdge(e1);

  auto extractVertexIds = [](const g2o::SparseOptimizer& graph) {
    std::vector<int> vertex_ids;
    for (const auto& id_v : graph.vertices()) vertex_ids.push_back(id_v.first);
    return vertex_ids;
  };

  auto extractEdgeIds = [](const g2o::SparseOptimizer& graph) {
    std::vector<std::pair<int, int>> edge_ids;
    for (const auto& e : graph.edges()) {
      const auto* edge = dynamic_cast<g2o::SparseOptimizer::Edge*>(e.get());
      edge_ids.emplace_back(edge->vertices()[0]->id(),
                            edge->vertices()[1]->id());
    }
    return edge_ids;
  };

  auto extractParamIds = [](const g2o::SparseOptimizer& graph) {
    std::vector<int> param_ids;
    for (const auto& p : graph.parameters()) param_ids.push_back(p.first);
    return param_ids;
  };

  const std::vector<int> param_ids = extractParamIds(*optimizer);
  const std::vector<int> vertex_ids = extractVertexIds(*optimizer);
  const std::vector<std::pair<int, int>> edge_ids = extractEdgeIds(*optimizer);

  g2o::SparseOptimizer fresh_graph;
  fresh_graph.addGraph(*optimizer);

  EXPECT_THAT(optimizer->vertices(), IsEmpty());
  EXPECT_THAT(optimizer->edges(), IsEmpty());
  EXPECT_THAT(extractParamIds(*optimizer), IsEmpty());

  EXPECT_THAT(extractParamIds(fresh_graph),
              UnorderedElementsAreArray(param_ids));
  EXPECT_THAT(extractVertexIds(fresh_graph),
              UnorderedElementsAreArray(vertex_ids));
  EXPECT_THAT(extractEdgeIds(fresh_graph), UnorderedElementsAreArray(edge_ids));
}

TEST(General, GraphChangeId) {
  std::unique_ptr<g2o::SparseOptimizer> optimizer(
      g2o::internal::createOptimizerForTests());

  auto v1 = std::make_shared<g2o::VertexSE2>();
  v1->setId(0);
  auto v2 = std::make_shared<g2o::VertexSE2>();
  v2->setId(1);
  optimizer->addVertex(v1);

  EXPECT_EQ(v1->id(), 0);
  auto aux = std::static_pointer_cast<g2o::HyperGraph::Vertex>(v1);
  const bool changed_id = optimizer->changeId(aux, 42);
  EXPECT_TRUE(changed_id);
  EXPECT_EQ(v1->id(), 42);

  EXPECT_EQ(v2->id(), 1);
  aux = std::static_pointer_cast<g2o::HyperGraph::Vertex>(v2);

  const bool not_changed = optimizer->changeId(aux, 17);
  EXPECT_FALSE(not_changed);
  EXPECT_EQ(v2->id(), 1);
}

class PublicRenamedTypesGraph : public g2o::OptimizableGraph {
 public:
  const std::unordered_map<std::string, std::string>& renamedTypesLookup()
      const {
    return renamedTypesLookup_;
  }
};

TEST(General, RenamedTypesFromString) {
  using namespace testing;  // NOLINT
  PublicRenamedTypesGraph optimizer;

  EXPECT_THAT(optimizer.renamedTypesLookup(), IsEmpty());

  optimizer.setRenamedTypesFromString("VERTEX_SE2:foobar");
  ASSERT_THAT(optimizer.renamedTypesLookup(), IsEmpty());

  optimizer.setRenamedTypesFromString("VERTEX_SE2:foobar=DoesNotExist");
  ASSERT_THAT(optimizer.renamedTypesLookup(), IsEmpty());

  optimizer.setRenamedTypesFromString("VERTEX_SE2:foobar=VERTEX_SE2");
  EXPECT_THAT(optimizer.renamedTypesLookup(), SizeIs(1));
  EXPECT_THAT(optimizer.renamedTypesLookup(),
              Contains(Key("VERTEX_SE2:foobar")));
  EXPECT_THAT(optimizer.renamedTypesLookup(),
              Contains(Pair("VERTEX_SE2:foobar", "VERTEX_SE2")));
}

TEST(General, SetEstimateData) {
  g2o::VertexSE2 vertex;

  std::vector<double> data_vector = {1., 2., 3.};
  vertex.setEstimateData(data_vector.data());
  EXPECT_THAT(
      print_wrap(vertex.estimate().toVector()),
      g2o::internal::EigenApproxEqual(print_wrap(g2o::Vector3(1, 2, 3)), 1e-3));

  vertex.setEstimateData(g2o::Vector3(3, 2, 1));
  EXPECT_THAT(
      print_wrap(vertex.estimate().toVector()),
      g2o::internal::EigenApproxEqual(print_wrap(g2o::Vector3(3, 2, 1)), 1e-3));

  vertex.setEstimateData(std::vector<double>({0.1, 0.2, 0.3}));
  EXPECT_THAT(print_wrap(vertex.estimate().toVector()),
              g2o::internal::EigenApproxEqual(
                  print_wrap(g2o::Vector3(0.1, 0.2, 0.3)), 1e-3));

  g2o::VectorX estimate_data;
  ASSERT_THAT(vertex.getEstimateData(estimate_data), IsTrue());
  EXPECT_THAT(print_wrap(estimate_data),
              g2o::internal::EigenApproxEqual(
                  print_wrap(g2o::Vector3(0.1, 0.2, 0.3)), 1e-3));

  vertex.setMinimalEstimateData(data_vector.data());
  EXPECT_THAT(
      print_wrap(vertex.estimate().toVector()),
      g2o::internal::EigenApproxEqual(print_wrap(g2o::Vector3(1, 2, 3)), 1e-3));

  vertex.setMinimalEstimateData(g2o::Vector3(3, 2, 1));
  EXPECT_THAT(
      print_wrap(vertex.estimate().toVector()),
      g2o::internal::EigenApproxEqual(print_wrap(g2o::Vector3(3, 2, 1)), 1e-3));

  vertex.setMinimalEstimateData(std::vector<double>({0.1, 0.2, 0.3}));
  EXPECT_THAT(print_wrap(vertex.estimate().toVector()),
              g2o::internal::EigenApproxEqual(
                  print_wrap(g2o::Vector3(0.1, 0.2, 0.3)), 1e-3));

  estimate_data.setZero();
  ASSERT_THAT(vertex.getMinimalEstimateData(estimate_data), IsTrue());
  EXPECT_THAT(print_wrap(estimate_data),
              g2o::internal::EigenApproxEqual(
                  print_wrap(g2o::Vector3(0.1, 0.2, 0.3)), 1e-3));
}

/**
 * Fixture to test saving and loading of a graph.
 * Here, we will have a simple graph with N nodes and N edges.
 * We use for saving and loading.
 */
class GeneralGraphOperations : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer_ = g2o::internal::createOptimizerForTests();

    // Add vertices
    for (size_t i = 0; i < kNumVertices; ++i) {
      auto v = std::make_shared<g2o::VertexSE2>();
      v->setEstimate(g2o::SE2());
      v->setId(i);
      v->setFixed(i == 0);  // fix the first vertex
      optimizer_->addVertex(v);
    }

    // Add edges
    for (size_t i = 0; i < kNumVertices; ++i) {
      auto e1 = std::make_shared<g2o::EdgeSE2>();
      e1->vertices()[0] = optimizer_->vertex((i + 0) % kNumVertices);
      e1->vertices()[1] = optimizer_->vertex((i + 1) % kNumVertices);
      e1->setMeasurement(g2o::SE2(1., 0., 0.));
      e1->setInformation(g2o::EdgeSE2::InformationType::Identity());
      optimizer_->addEdge(e1);
    }
  }

  // void TearDown() override {}

  //! recover the vertex IDs from a output generated by OptimizableGraph::save()
  static std::vector<int> parseVertexIds(std::istream& is) {
    std::vector<int> result;
    std::stringstream currentLine;
    while (g2o::readLine(is, currentLine) >= 0) {
      std::string token;
      currentLine >> token;
      if (g2o::strStartsWith(token, "VERTEX_")) {
        int id;
        currentLine >> id;
        result.push_back(id);
      }
    }
    return result;
  }

  //! recover the fixed IDs from a output generated by OptimizableGraph::save()
  static std::vector<int> parseFixedIds(std::istream& is) {
    std::vector<int> result;
    std::stringstream currentLine;
    while (g2o::readLine(is, currentLine) >= 0) {
      std::string token;
      currentLine >> token;
      if (token == "FIX") {
        int id;
        while (currentLine >> id) result.push_back(id);
      }
    }
    return result;
  }

  //! recover the vertex IDs of the edges from a output generated by
  //! OptimizableGraph::save()
  static std::vector<std::pair<int, int>> parseEdgeIds(std::istream& is) {
    std::vector<std::pair<int, int>> result;
    std::stringstream currentLine;
    while (g2o::readLine(is, currentLine) >= 0) {
      std::string token;
      currentLine >> token;
      if (g2o::strStartsWith(token, "EDGE_")) {
        int id1;
        int id2;
        currentLine >> id1 >> id2;
        result.emplace_back(id1, id2);
      }
    }
    return result;
  }

  //! helper method to restart reading at the start of a stream
  static void resetStream(std::istream& is) {
    is.clear();
    is.seekg(0, std::ios::beg);
  }

  //! returns the expected Vertex IDs of the fixture
  static std::vector<int> expectedIds() {
    std::vector<int> result(kNumVertices);
    std::iota(result.begin(), result.end(), 0);
    return result;
  }

  //! returns the expected Vertex IDs of the edges of the fixture
  static std::vector<std::pair<int, int>> expectedEdgeIds() {
    std::vector<std::pair<int, int>> result;
    for (size_t i = 0; i < kNumVertices; ++i)
      result.emplace_back((i + 0) % kNumVertices, (i + 1) % kNumVertices);
    return result;
  }

  [[nodiscard]] std::map<int, g2o::Vector3> vertexEstimates() const {
    std::map<int, g2o::Vector3> result;
    for (const auto& idV : optimizer_->vertices()) {
      auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
      result[idV.first] = v->estimate().toVector();
    }
    return result;
  }

  //! returns the expected Vertex IDs of the fixture
  [[nodiscard]] std::vector<int> fixedIds() const {
    std::vector<int> result;
    for (const auto& idV : optimizer_->vertices()) {
      auto* v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(idV.second.get());
      if (v->fixed()) result.push_back(v->id());
    }
    return result;
  }

  std::shared_ptr<g2o::SparseOptimizer> optimizer_;
  static constexpr size_t kNumVertices = 3;
};

constexpr size_t GeneralGraphOperations::kNumVertices;

TEST_F(GeneralGraphOperations, SavingGraph) {
  std::stringstream graphData;
  optimizer_->save(graphData);

  // Analyze the written graph data
  resetStream(graphData);
  auto vertexIds = parseVertexIds(graphData);
  resetStream(graphData);
  auto fixedIds = parseFixedIds(graphData);
  resetStream(graphData);
  auto edgeIds = parseEdgeIds(graphData);

  EXPECT_THAT(vertexIds, testing::ElementsAreArray(expectedIds()));
  EXPECT_THAT(fixedIds, testing::ElementsAre(0));
  EXPECT_THAT(edgeIds, testing::ElementsAreArray(expectedEdgeIds()));
}

namespace {
using KeyIntVector = std::vector<decltype(testing::Key(42))>;
static KeyIntVector VectorIntToKeys(const std::vector<int>& keys) {  // NOLINT
  KeyIntVector matchers;
  for (const auto& val : keys) matchers.push_back(testing::Key(val));
  return matchers;
}
}  // namespace

TEST_F(GeneralGraphOperations, LoadingGraph) {
  std::stringstream graphData;
  optimizer_->save(graphData);

  // clear the graph and make sure it is actually empty
  optimizer_->clear();
  ASSERT_THAT(optimizer_->vertices(), testing::SizeIs(0));
  ASSERT_THAT(optimizer_->edges(), testing::SizeIs(0));

  // load the graph and test the loaded vertices and edges
  optimizer_->load(graphData);
  ASSERT_THAT(optimizer_->vertices(), testing::SizeIs(kNumVertices));
  ASSERT_THAT(optimizer_->edges(), testing::SizeIs(kNumVertices));
  ASSERT_THAT(optimizer_->dimensions(), testing::ElementsAre(3));

  ASSERT_THAT(optimizer_->vertices(), testing::UnorderedElementsAreArray(
                                          VectorIntToKeys(expectedIds())));
  ASSERT_THAT(
      optimizer_->edges(),
      testing::Each(testing::Pointee(testing::Property(
          &g2o::OptimizableGraph::Edge::vertices, testing::SizeIs(2)))));
  ASSERT_THAT(
      optimizer_->edges(),
      testing::Each(testing::ResultOf(
          [](const std::shared_ptr<g2o::HyperGraph::Edge>& e) {
            return static_cast<g2o::OptimizableGraph::Edge*>(e.get())->graph();
          },
          testing::Eq(static_cast<g2o::OptimizableGraph*>(optimizer_.get())))));
  ASSERT_THAT(optimizer_->edges(),
              testing::Each(testing::ResultOf(
                  [](const std::shared_ptr<g2o::HyperGraph::Edge>& e) {
                    return std::make_pair(e->vertex(0)->id(),
                                          e->vertex(1)->id());
                  },
                  testing::AnyOfArray(expectedEdgeIds()))));
}

TEST_F(GeneralGraphOperations, SaveSubsetVertices) {
  g2o::OptimizableGraph::VertexSet verticesToSave{optimizer_->vertex(0),
                                                  optimizer_->vertex(1)};

  std::stringstream graphData;
  optimizer_->saveSubset(graphData, verticesToSave);
  optimizer_->clear();

  optimizer_->load(graphData);
  EXPECT_THAT(optimizer_->vertices(), testing::SizeIs(2));
  EXPECT_THAT(optimizer_->edges(), testing::SizeIs(1));
  EXPECT_THAT(optimizer_->vertices(),
              testing::UnorderedElementsAreArray(VectorIntToKeys({0, 1})));
}

TEST_F(GeneralGraphOperations, SaveSubsetEdges) {
  g2o::OptimizableGraph::EdgeSet edgesToSave;
  std::copy_if(optimizer_->edges().begin(), optimizer_->edges().end(),
               std::inserter(edgesToSave, edgesToSave.end()),
               [](const g2o::HyperGraph::EdgeSet::value_type& e) {
                 return e->vertices().size() == 2 && e->vertex(0)->id() == 1 &&
                        e->vertex(1)->id() == 2;
               });

  ASSERT_THAT(edgesToSave, testing::SizeIs(1));

  std::stringstream graphData;
  optimizer_->saveSubset(graphData, edgesToSave);
  optimizer_->clear();

  optimizer_->load(graphData);
  EXPECT_THAT(optimizer_->vertices(), testing::SizeIs(2));
  EXPECT_THAT(optimizer_->edges(), testing::SizeIs(1));
  EXPECT_THAT(optimizer_->vertices(),
              testing::UnorderedElementsAreArray(VectorIntToKeys({1, 2})));
}

TEST_F(GeneralGraphOperations, PushPopActiveVertices) {
  optimizer_->initializeOptimization();
  const std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer_->push();
  for (const auto& idV : optimizer_->vertices()) {
    auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    v->setEstimate(v->estimate() * g2o::SE2(idV.first, 0, 0));
    ASSERT_THAT(v->stackSize(), testing::Eq(1));
  }
  // the estimates should differ now
  const std::map<int, g2o::Vector3> changedEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(changedEstimates));
  // recover
  optimizer_->pop();
  const std::map<int, g2o::Vector3> recoveredEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Eq(recoveredEstimates));
  for (const auto& idV : optimizer_->vertices()) {
    auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushDiscardActiveVertices) {
  optimizer_->initializeOptimization();
  const std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer_->push();
  for (const auto& idV : optimizer_->vertices()) {
    auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    v->setEstimate(v->estimate() * g2o::SE2(idV.first, 0, 0));
    ASSERT_THAT(v->stackSize(), testing::Eq(1));
  }
  // the estimates should differ now
  const std::map<int, g2o::Vector3> changedEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(changedEstimates));
  // recover
  optimizer_->discardTop();
  const std::map<int, g2o::Vector3> recoveredEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(recoveredEstimates));
  ASSERT_THAT(changedEstimates, testing::Eq(recoveredEstimates));
  for (const auto& idV : optimizer_->vertices()) {
    auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushPopOptimizableGraph) {
  const std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer_->OptimizableGraph::push();
  for (const auto& idV : optimizer_->vertices()) {
    auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    v->setEstimate(v->estimate() * g2o::SE2(idV.first * 2, 0, 0));
    ASSERT_THAT(v->stackSize(), testing::Eq(1));
  }
  // the estimates should differ now
  const std::map<int, g2o::Vector3> changedEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(changedEstimates));
  // recover
  optimizer_->OptimizableGraph::pop();
  const std::map<int, g2o::Vector3> recoveredEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Eq(recoveredEstimates));
  for (const auto& idV : optimizer_->vertices()) {
    auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushDiscardOptimizableGraph) {
  optimizer_->initializeOptimization();
  const std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer_->OptimizableGraph::push();
  for (const auto& idV : optimizer_->vertices()) {
    auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    v->setEstimate(v->estimate() * g2o::SE2(idV.first * 3, 0, 0));
    ASSERT_THAT(v->stackSize(), testing::Eq(1));
  }
  // the estimates should differ now
  const std::map<int, g2o::Vector3> changedEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(changedEstimates));
  // recover
  optimizer_->OptimizableGraph::discardTop();
  const std::map<int, g2o::Vector3> recoveredEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(recoveredEstimates));
  ASSERT_THAT(changedEstimates, testing::Eq(recoveredEstimates));
  for (const auto& idV : optimizer_->vertices()) {
    auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushPopDiscardSubset) {
  g2o::HyperGraph::VertexSet pushingSet = {optimizer_->vertex(0),
                                           optimizer_->vertex(2)};
  g2o::HyperGraph::VertexSet notPushedSet;
  for (const auto& idV : optimizer_->vertices()) {
    if (pushingSet.count(idV.second) == 0) notPushedSet.insert(idV.second);
  }

  for (int operation = 0; operation < 2; ++operation) {
    // push
    optimizer_->push(pushingSet);
    for (const auto& elem : pushingSet) {
      auto* v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(elem.get());
      ASSERT_THAT(v->stackSize(), testing::Eq(1));
    }
    for (const auto& elem : notPushedSet) {
      auto* v = dynamic_cast<g2o::OptimizableGraph::Vertex*>(elem.get());
      ASSERT_THAT(v->stackSize(), testing::Eq(0));
    }
    // pop or discard
    if (operation == 0)
      optimizer_->pop(pushingSet);
    else
      optimizer_->discardTop(pushingSet);
    for (const auto& idV : optimizer_->vertices()) {
      auto* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
      ASSERT_THAT(v->stackSize(), testing::Eq(0));
    }
  }
}

TEST_F(GeneralGraphOperations, FixSubset) {
  EXPECT_THAT(fixedIds(),
              testing::ElementsAre(0));  // just vertex with ID zero is fixed

  g2o::HyperGraph::VertexSet fixingSet;
  fixingSet.insert(optimizer_->vertex(1));

  optimizer_->setFixed(fixingSet, true);
  EXPECT_THAT(fixedIds(), testing::WhenSorted(testing::ElementsAre(0, 1)));

  optimizer_->setFixed(fixingSet, false);
  EXPECT_THAT(fixedIds(), testing::ElementsAre(0));
}

TEST_F(GeneralGraphOperations, TrivialInit) {
  optimizer_->initializeOptimization();

  optimizer_->computeActiveErrors();
  ASSERT_THAT(optimizer_->activeVertices(), testing::SizeIs(kNumVertices));
  ASSERT_THAT(optimizer_->activeChi2(), testing::Gt(0.));
  ASSERT_TRUE(optimizer_->verifyInformationMatrices());

  ASSERT_THAT(
      optimizer_->activeVertices(),
      testing::Each(testing::ResultOf(
          [this](const std::shared_ptr<g2o::OptimizableGraph::Vertex>& v) {
            return optimizer_->findActiveVertex(v.get());
          },
          testing::Not(testing::Eq(optimizer_->activeVertices().end())))));

  std::map<int, g2o::Vector3> estimates = vertexEstimates();
  for (size_t i = 1; i < kNumVertices; ++i)
    ASSERT_DOUBLE_EQ(0, estimates[i].norm());
  optimizer_->computeInitialGuess();
  estimates = vertexEstimates();
  for (size_t i = 1; i < kNumVertices; ++i) ASSERT_LT(0, estimates[i].norm());
}

TEST_F(GeneralGraphOperations, EdgeInit) {
  auto e = std::dynamic_pointer_cast<g2o::OptimizableGraph::Edge>(
      *optimizer_->edges().begin());
  auto e2 = std::dynamic_pointer_cast<g2o::OptimizableGraph::Edge>(
      *std::next(optimizer_->edges().begin()));
  g2o::HyperGraph::EdgeSet eset = {e};

  optimizer_->initializeOptimization(eset);

  EXPECT_THAT(optimizer_->activeVertices(), testing::SizeIs(2));
  EXPECT_THAT(optimizer_->activeEdges(), testing::SizeIs(1));
  EXPECT_THAT(optimizer_->findActiveEdge(e.get()),
              testing::Not(testing::Eq(optimizer_->activeEdges().end())));
  EXPECT_THAT(optimizer_->findActiveEdge(e2.get()),
              testing::Eq(optimizer_->activeEdges().end()));
}

TEST_F(GeneralGraphOperations, Gauge) {
  optimizer_->initializeOptimization();

  EXPECT_THAT(optimizer_->gaugeFreedom(), testing::IsFalse());
  EXPECT_THAT(optimizer_->findGauge(), testing::NotNull());

  optimizer_->vertex(0)->setFixed(false);
  EXPECT_THAT(optimizer_->gaugeFreedom(), testing::IsTrue());
  EXPECT_THAT(optimizer_->findGauge(), testing::NotNull());
}

TEST_F(GeneralGraphOperations, Dimensions) {
  optimizer_->initializeOptimization();

  EXPECT_EQ(optimizer_->maxDimension(), 3);
  EXPECT_THAT(optimizer_->dimensions(), testing::ElementsAre(3));

  auto point = std::make_shared<g2o::VertexPointXY>();
  point->setId(kNumVertices + 1);
  optimizer_->addVertex(point);

  EXPECT_EQ(optimizer_->maxDimension(), 3);
  EXPECT_THAT(optimizer_->dimensions(), testing::ElementsAre(2, 3));
}

TEST_F(GeneralGraphOperations, VerifyInformationMatrices) {
  optimizer_->initializeOptimization();

  ASSERT_TRUE(optimizer_->verifyInformationMatrices());

  auto e1 = std::make_shared<g2o::EdgeSE2>();
  e1->vertices()[0] = optimizer_->vertex(0);
  e1->vertices()[1] = optimizer_->vertex(1);
  e1->setMeasurement(g2o::SE2(2., 0., 0.));
  e1->setInformation(-1 * g2o::EdgeSE2::InformationType::Identity());
  optimizer_->addEdge(e1);

  auto e2 = std::make_shared<g2o::EdgeSE2>();
  e2->vertices()[0] = optimizer_->vertex(1);
  e2->vertices()[1] = optimizer_->vertex(2);
  e2->setMeasurement(g2o::SE2(1., 2., 0.));
  g2o::EdgeSE2::InformationType infoMat =
      g2o::EdgeSE2::InformationType::Identity();
  infoMat(1, 2) = 1;
  e2->setInformation(infoMat);
  optimizer_->addEdge(e2);

  ASSERT_FALSE(optimizer_->verifyInformationMatrices(true));
}

TEST_F(GeneralGraphOperations, SolverSuitable) {
  g2o::OptimizationAlgorithmProperty solverPropertyVar;
  solverPropertyVar.requiresMarginalize = false;
  solverPropertyVar.poseDim = -1;
  solverPropertyVar.landmarkDim = -1;

  g2o::OptimizationAlgorithmProperty solverPropertyFix32;
  solverPropertyFix32.requiresMarginalize = true;
  solverPropertyFix32.poseDim = 3;
  solverPropertyFix32.landmarkDim = 2;

  g2o::OptimizationAlgorithmProperty solverPropertyFix63;
  solverPropertyFix63.requiresMarginalize = true;
  solverPropertyFix63.poseDim = 6;
  solverPropertyFix63.landmarkDim = 3;

  const std::set<int> vertexDims = {2, 3};
  const std::set<int> vertexDimsNoMatch = {1, 5};

  EXPECT_TRUE(optimizer_->isSolverSuitable(solverPropertyVar));
  EXPECT_TRUE(optimizer_->isSolverSuitable(solverPropertyVar, vertexDims));
  EXPECT_TRUE(
      optimizer_->isSolverSuitable(solverPropertyVar, vertexDimsNoMatch));

  EXPECT_TRUE(optimizer_->isSolverSuitable(solverPropertyFix32));
  EXPECT_TRUE(optimizer_->isSolverSuitable(solverPropertyFix32, vertexDims));
  EXPECT_FALSE(
      optimizer_->isSolverSuitable(solverPropertyFix32, vertexDimsNoMatch));

  EXPECT_FALSE(optimizer_->isSolverSuitable(solverPropertyFix63));
  EXPECT_FALSE(optimizer_->isSolverSuitable(solverPropertyFix63, vertexDims));
  EXPECT_FALSE(
      optimizer_->isSolverSuitable(solverPropertyFix63, vertexDimsNoMatch));
}

TEST_F(GeneralGraphOperations, SharedOwnerShip) {
  using Ptr = std::shared_ptr<g2o::HyperGraph::Vertex>;
  using IdVertexPair = std::pair<int, Ptr>;

  g2o::HyperGraph::VertexIDMap verticesMap = optimizer_->vertices();
  optimizer_->clear();

  // currently vertices are owned by our array and the edges
  ASSERT_THAT(
      verticesMap,
      testing::Each(Field(&IdVertexPair::second,
                          testing::Property(&Ptr::use_count, testing::Ge(1)))));

  // clear all edges of each vertex
  for (auto& v : verticesMap) v.second->edges().clear();
  ASSERT_THAT(
      verticesMap,
      testing::Each(Field(&IdVertexPair::second,
                          testing::Property(&Ptr::use_count, testing::Ge(1)))));

  const Ptr singleVertex = verticesMap.begin()->second;
  verticesMap.clear();
  ASSERT_THAT(singleVertex.use_count(), testing::Eq(1));
}

TEST_F(GeneralGraphOperations, JacWorkspace) {
  JacobianWorkspaceTestAdapter workspace;
  ASSERT_THAT(workspace.maxDimension(), testing::Le(0));
  ASSERT_THAT(workspace.maxNumVertices(), testing::Le(0));

  auto root = optimizer_->vertex(0);
  std::shared_ptr<g2o::HyperGraph::Edge> first_edge =
      root->edges().begin()->lock();
  workspace.updateSize(*first_edge, true);
  EXPECT_THAT(workspace.maxDimension(), testing::Eq(9));
  EXPECT_THAT(workspace.maxNumVertices(), testing::Eq(2));
  EXPECT_THAT(workspace.workspace(), testing::IsEmpty());

  workspace.updateSize(5, 23, true);
  EXPECT_THAT(workspace.maxDimension(), testing::Eq(23));
  EXPECT_THAT(workspace.maxNumVertices(), testing::Eq(5));
  EXPECT_THAT(workspace.workspace(), testing::IsEmpty());

  workspace.updateSize(*optimizer_, true);
  EXPECT_THAT(workspace.maxDimension(), testing::Eq(9));
  EXPECT_THAT(workspace.maxNumVertices(), testing::Eq(2));
  EXPECT_THAT(workspace.workspace(), testing::IsEmpty());

  bool allocated = workspace.allocate();
  EXPECT_THAT(allocated, testing::IsTrue());
  ASSERT_THAT(workspace.workspace(), testing::SizeIs(2));
  EXPECT_THAT(workspace.workspace(),
              testing::Each(testing::SizeIs(workspace.maxDimension())));

  for (int i = 0; i < 2; ++i) {
    EXPECT_THAT(workspace.workspaceForVertex(i), testing::NotNull());
  }

  for (auto& wp : workspace.workspace()) wp.array() = 1;
  workspace.setZero();
  EXPECT_THAT(
      workspace.workspace(),
      testing::Each(testing::ResultOf(
          [](const g2o::VectorX& vec) { return vec.isApproxToConstant(0); },
          testing::IsTrue())));
}

namespace {

class TreeVisitor : public g2o::HyperDijkstra::TreeAction {
 public:
  double perform(
      const std::shared_ptr<g2o::HyperGraph::Vertex>& v,
      const std::shared_ptr<g2o::HyperGraph::Vertex>& /*parent*/,
      const std::shared_ptr<g2o::HyperGraph::Edge>& /*edge*/) override {
    visited_ids_.insert(v->id());
    return 1.;
  }

  const std::unordered_set<int>& visitedIds() const { return visited_ids_; }

 protected:
  std::unordered_set<int> visited_ids_;
};

std::vector<int> range_helper(int range) {
  std::vector<int> result(range);
  std::iota(result.begin(), result.end(), 0);
  return result;
}
}  // namespace

TEST_F(GeneralGraphOperations, HyperDijkstraVisitor) {
  g2o::UniformCostFunction uniformCost;
  g2o::HyperDijkstra hyperDijkstra(optimizer_);
  hyperDijkstra.shortestPaths(optimizer_->vertex(0), uniformCost);

  g2o::HyperDijkstra::computeTree(hyperDijkstra.adjacencyMap());
  TreeVisitor treeVisitor;
  g2o::HyperDijkstra::visitAdjacencyMap(hyperDijkstra.adjacencyMap(),
                                        treeVisitor);
  EXPECT_THAT(treeVisitor.visitedIds(), testing::SizeIs(kNumVertices));
  EXPECT_THAT(treeVisitor.visitedIds(),
              testing::UnorderedElementsAreArray(range_helper(kNumVertices)));
}
