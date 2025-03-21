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
#include <memory>
#include <numeric>
#include <unordered_set>

#include "allocate_optimizer.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/types/slam2d/types_slam2d.h"  // IWYU pragma: keep
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"

G2O_USE_TYPE_GROUP(slam2d);

namespace {
class JacobianWorkspaceTestAdapter : public g2o::JacobianWorkspace {
 public:
  [[nodiscard]] WorkspaceVector& workspace() { return _workspace; }
  [[nodiscard]] int maxNumVertices() const { return _maxNumVertices; }
  [[nodiscard]] int maxDimension() const { return _maxDimension; }
};
}  // namespace

TEST(General, BinaryEdgeConstructor) {
  g2o::EdgeSE2 e2;
  ASSERT_EQ(nullptr, e2.vertices()[0]);
  ASSERT_EQ(nullptr, e2.vertices()[1]);
}

TEST(General, GraphAddVertex) {
  g2o::SparseOptimizer* optimizer = g2o::internal::createOptimizerForTests();

  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_EQ(optimizer, v1->graph());
  ASSERT_EQ(size_t(1), optimizer->vertices().size());
  ASSERT_FALSE(optimizer->addVertex(v1));
  ASSERT_EQ(size_t(1), optimizer->vertices().size());

  auto* v1_from_optimizer = optimizer->vertex(0);
  ASSERT_EQ(v1, v1_from_optimizer);

  {
    g2o::VertexSE2* v2 = new g2o::VertexSE2();
    v2->setId(0);
    ASSERT_FALSE(optimizer->addVertex(v2));
    ASSERT_EQ(size_t(1), optimizer->vertices().size());
    ASSERT_EQ(nullptr, v2->graph());
    delete v2;
  }

  const bool removed = optimizer->removeVertex(v1, true);
  ASSERT_TRUE(removed);
  ASSERT_TRUE(optimizer->vertices().empty());

  delete optimizer;
}

TEST(General, GraphAddEdge) {
  g2o::SparseOptimizer* optimizer = g2o::internal::createOptimizerForTests();

  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  g2o::VertexSE2* v2 = new g2o::VertexSE2();
  v2->setId(1);

  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_TRUE(optimizer->addVertex(v2));

  g2o::EdgeSE2* e1 = new g2o::EdgeSE2();
  e1->setVertex(0, v1);
  e1->setVertex(1, v2);
  ASSERT_TRUE(optimizer->addEdge(e1));
  ASSERT_EQ(optimizer, e1->graph());
  ASSERT_FALSE(optimizer->addEdge(e1));
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().count(e1));
  ASSERT_EQ(size_t(1), v2->edges().count(e1));

  g2o::EdgeSE2* e2 = new g2o::EdgeSE2();
  ASSERT_FALSE(optimizer->addEdge(e2))
      << "Adding edge with unset vertices was possible";
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(nullptr, e2->graph());

  g2o::EdgeSE2* e3 = new g2o::EdgeSE2();
  e3->setVertex(0, v1);
  e3->setVertex(1, v1);
  ASSERT_FALSE(optimizer->addEdge(e3))
      << "Adding binary edge with same vertices was possible";
  ASSERT_EQ(size_t(1), optimizer->edges().size());

  const bool removed = optimizer->removeEdge(e1);
  ASSERT_TRUE(removed);
  ASSERT_TRUE(optimizer->edges().empty());

  delete e2;
  delete e3;
  delete optimizer;
}

TEST(General, GraphAddEdgeNoRequiredParam) {
  g2o::SparseOptimizer* optimizer = g2o::internal::createOptimizerForTests();

  g2o::VertexSE3* poseVertex = new g2o::VertexSE3();
  poseVertex->setId(0);
  optimizer->addVertex(poseVertex);

  g2o::VertexPointXYZ* pointVertex = new g2o::VertexPointXYZ();
  pointVertex->setId(1);
  optimizer->addVertex(pointVertex);

  g2o::EdgeSE3PointXYZ* edge = new g2o::EdgeSE3PointXYZ();
  edge->setVertex(0, poseVertex);
  edge->setVertex(1, pointVertex);

  EXPECT_FALSE(optimizer->addEdge(edge));

  optimizer->clear();
  delete optimizer;
}

TEST(General, GraphChangeId) {
  std::unique_ptr<g2o::SparseOptimizer> optimizer(
      g2o::internal::createOptimizerForTests());

  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  g2o::VertexSE2 v2;
  v2.setId(1);
  optimizer->addVertex(v1);

  EXPECT_EQ(v1->id(), 0);
  const bool changed_id = optimizer->changeId(v1, 42);
  EXPECT_TRUE(changed_id);
  EXPECT_EQ(v1->id(), 42);

  EXPECT_EQ(v2.id(), 1);
  const bool not_changed = optimizer->changeId(&v2, 17);
  EXPECT_FALSE(not_changed);
  EXPECT_EQ(v2.id(), 1);
}

/**
 * Fixture to test saving and loading of a graph.
 * Here, we will have a simple graph with N nodes and N edges.
 * We use for saving and loading.
 */
class GeneralGraphOperations : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer.reset(g2o::internal::createOptimizerForTests());

    // Add vertices
    for (size_t i = 0; i < numVertices; ++i) {
      g2o::VertexSE2* v = new g2o::VertexSE2;
      v->setEstimate(g2o::SE2());
      v->setId(i);
      v->setFixed(i == 0);  // fix the first vertex
      optimizer->addVertex(v);
    }

    // Add edges
    for (size_t i = 0; i < numVertices; ++i) {
      g2o::EdgeSE2* e1 = new g2o::EdgeSE2();
      e1->vertices()[0] = optimizer->vertex((i + 0) % numVertices);
      e1->vertices()[1] = optimizer->vertex((i + 1) % numVertices);
      e1->setMeasurement(g2o::SE2(1., 0., 0.));
      e1->setInformation(g2o::EdgeSE2::InformationType::Identity());
      optimizer->addEdge(e1);
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

  //! recover the vertec IDs of the edges from a output generated by
  //! OptimizableGraph::save()
  static std::vector<std::pair<int, int>> parseEdgeIds(std::istream& is) {
    std::vector<std::pair<int, int>> result;
    std::stringstream currentLine;
    while (g2o::readLine(is, currentLine) >= 0) {
      std::string token;
      currentLine >> token;
      if (g2o::strStartsWith(token, "EDGE_")) {
        int id1, id2;
        currentLine >> id1 >> id2;
        result.emplace_back(std::make_pair(id1, id2));
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
  std::vector<int> expectedIds() const {
    std::vector<int> result(numVertices);
    std::iota(result.begin(), result.end(), 0);
    return result;
  }

  //! returns the expected Vertex IDs of the edges of the fixture
  std::vector<std::pair<int, int>> expectedEdgeIds() const {
    std::vector<std::pair<int, int>> result;
    for (size_t i = 0; i < numVertices; ++i)
      result.emplace_back((i + 0) % numVertices, (i + 1) % numVertices);
    return result;
  }

  std::map<int, g2o::Vector3> vertexEstimates() const {
    std::map<int, g2o::Vector3> result;
    for (const auto& idV : optimizer->vertices()) {
      g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
      result[idV.first] = v->estimate().toVector();
    }
    return result;
  }

  //! returns the expected Vertex IDs of the fixture
  std::vector<int> fixedIds() const {
    std::vector<int> result;
    for (const auto& idV : optimizer->vertices()) {
      g2o::OptimizableGraph::Vertex* v =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(idV.second);
      if (v->fixed()) result.push_back(v->id());
    }
    return result;
  }

  std::unique_ptr<g2o::SparseOptimizer> optimizer;
  size_t numVertices = 3;
};

TEST_F(GeneralGraphOperations, SavingGraph) {
  std::stringstream graphData;
  optimizer->save(graphData);

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

namespace internal {
using KeyIntVector = std::vector<decltype(testing::Key(42))>;
static KeyIntVector VectorIntToKeys(const std::vector<int>& keys) {
  KeyIntVector matchers;
  for (const auto& val : keys) matchers.push_back(testing::Key(val));
  return matchers;
}
}  // namespace internal

TEST_F(GeneralGraphOperations, LoadingGraph) {
  std::stringstream graphData;
  optimizer->save(graphData);

  // clear the graph and make sure it is actually empty
  optimizer->clear();
  ASSERT_THAT(optimizer->vertices(), testing::SizeIs(0));
  ASSERT_THAT(optimizer->edges(), testing::SizeIs(0));

  // load the graph and test the loaded vertices and edges
  optimizer->load(graphData);
  ASSERT_THAT(optimizer->vertices(), testing::SizeIs(numVertices));
  ASSERT_THAT(optimizer->edges(), testing::SizeIs(numVertices));
  ASSERT_THAT(optimizer->dimensions(), testing::ElementsAre(3));

  ASSERT_THAT(optimizer->vertices(),
              testing::UnorderedElementsAreArray(
                  internal::VectorIntToKeys(expectedIds())));
  ASSERT_THAT(optimizer->edges(),
              testing::Each(testing::Property(
                  &g2o::OptimizableGraph::Edge::vertices, testing::SizeIs(2))));
  ASSERT_THAT(
      optimizer->edges(),
      testing::Each(testing::ResultOf(
          [](g2o::HyperGraph::Edge* e) {
            return static_cast<g2o::OptimizableGraph::Edge*>(e)->graph();
          },
          testing::Eq(static_cast<g2o::OptimizableGraph*>(optimizer.get())))));
  ASSERT_THAT(optimizer->edges(), testing::Each(testing::ResultOf(
                                      [](g2o::HyperGraph::Edge* e) {
                                        return std::make_pair(
                                            e->vertex(0)->id(),
                                            e->vertex(1)->id());
                                      },
                                      testing::AnyOfArray(expectedEdgeIds()))));
}

TEST_F(GeneralGraphOperations, SaveSubsetVertices) {
  g2o::OptimizableGraph::VertexSet verticesToSave{optimizer->vertex(0),
                                                  optimizer->vertex(1)};

  std::stringstream graphData;
  optimizer->saveSubset(graphData, verticesToSave);
  optimizer->clear();

  optimizer->load(graphData);
  EXPECT_THAT(optimizer->vertices(), testing::SizeIs(2));
  EXPECT_THAT(optimizer->edges(), testing::SizeIs(1));
  EXPECT_THAT(optimizer->vertices(), testing::UnorderedElementsAreArray(
                                         internal::VectorIntToKeys({0, 1})));
}

TEST_F(GeneralGraphOperations, SaveSubsetEdges) {
  g2o::OptimizableGraph::EdgeSet edgesToSave;
  std::copy_if(optimizer->edges().begin(), optimizer->edges().end(),
               std::inserter(edgesToSave, edgesToSave.end()),
               [](const g2o::HyperGraph::Edge* e) {
                 return e->vertices().size() == 2 && e->vertex(0)->id() == 1 &&
                        e->vertex(1)->id() == 2;
               });

  ASSERT_THAT(edgesToSave, testing::SizeIs(1));

  std::stringstream graphData;
  optimizer->saveSubset(graphData, edgesToSave);
  optimizer->clear();

  optimizer->load(graphData);
  EXPECT_THAT(optimizer->vertices(), testing::SizeIs(2));
  EXPECT_THAT(optimizer->edges(), testing::SizeIs(1));
  EXPECT_THAT(optimizer->vertices(), testing::UnorderedElementsAreArray(
                                         internal::VectorIntToKeys({1, 2})));
}

TEST_F(GeneralGraphOperations, PushPopActiveVertices) {
  optimizer->initializeOptimization();
  std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer->push();
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
    v->setEstimate(v->estimate() * g2o::SE2(idV.first, 0, 0));
    ASSERT_THAT(v->stackSize(), testing::Eq(1));
  }
  // the estimates should differ now
  std::map<int, g2o::Vector3> changedEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(changedEstimates));
  // recover
  optimizer->pop();
  std::map<int, g2o::Vector3> recoveredEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Eq(recoveredEstimates));
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushDiscardActiveVertices) {
  optimizer->initializeOptimization();
  std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer->push();
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
    v->setEstimate(v->estimate() * g2o::SE2(idV.first, 0, 0));
    ASSERT_THAT(v->stackSize(), testing::Eq(1));
  }
  // the estimates should differ now
  std::map<int, g2o::Vector3> changedEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(changedEstimates));
  // recover
  optimizer->discardTop();
  std::map<int, g2o::Vector3> recoveredEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(recoveredEstimates));
  ASSERT_THAT(changedEstimates, testing::Eq(recoveredEstimates));
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushPopOptimizableGraph) {
  std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer->OptimizableGraph::push();
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
    v->setEstimate(v->estimate() * g2o::SE2(idV.first * 2, 0, 0));
    ASSERT_THAT(v->stackSize(), testing::Eq(1));
  }
  // the estimates should differ now
  std::map<int, g2o::Vector3> changedEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(changedEstimates));
  // recover
  optimizer->OptimizableGraph::pop();
  std::map<int, g2o::Vector3> recoveredEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Eq(recoveredEstimates));
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushDiscardOptimizableGraph) {
  optimizer->initializeOptimization();
  std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer->OptimizableGraph::push();
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
    v->setEstimate(v->estimate() * g2o::SE2(idV.first * 3, 0, 0));
    ASSERT_THAT(v->stackSize(), testing::Eq(1));
  }
  // the estimates should differ now
  std::map<int, g2o::Vector3> changedEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(changedEstimates));
  // recover
  optimizer->OptimizableGraph::discardTop();
  std::map<int, g2o::Vector3> recoveredEstimates = vertexEstimates();
  ASSERT_THAT(originalEstimates, testing::Ne(recoveredEstimates));
  ASSERT_THAT(changedEstimates, testing::Eq(recoveredEstimates));
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushPopDiscardSubset) {
  g2o::HyperGraph::VertexSet pushingSet = {optimizer->vertex(0),
                                           optimizer->vertex(2)};
  g2o::HyperGraph::VertexSet notPushedSet;
  for (const auto& idV : optimizer->vertices()) {
    if (pushingSet.count(idV.second) == 0) notPushedSet.insert(idV.second);
  }

  for (int operation = 0; operation < 2; ++operation) {
    // push
    optimizer->push(pushingSet);
    for (const auto& elem : pushingSet) {
      g2o::OptimizableGraph::Vertex* v =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(elem);
      ASSERT_THAT(v->stackSize(), testing::Eq(1));
    }
    for (const auto& elem : notPushedSet) {
      g2o::OptimizableGraph::Vertex* v =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(elem);
      ASSERT_THAT(v->stackSize(), testing::Eq(0));
    }
    // pop or discard
    if (operation == 0)
      optimizer->pop(pushingSet);
    else
      optimizer->discardTop(pushingSet);
    for (const auto& idV : optimizer->vertices()) {
      g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second);
      ASSERT_THAT(v->stackSize(), testing::Eq(0));
    }
  }
}

TEST_F(GeneralGraphOperations, FixSubset) {
  EXPECT_THAT(fixedIds(),
              testing::ElementsAre(0));  // just vertex with ID zero is fixed

  g2o::HyperGraph::VertexSet fixingSet;
  fixingSet.insert(optimizer->vertex(1));

  optimizer->setFixed(fixingSet, true);
  EXPECT_THAT(fixedIds(), testing::WhenSorted(testing::ElementsAre(0, 1)));

  optimizer->setFixed(fixingSet, false);
  EXPECT_THAT(fixedIds(), testing::ElementsAre(0));
}

TEST_F(GeneralGraphOperations, TrivialInit) {
  optimizer->initializeOptimization();

  optimizer->computeActiveErrors();
  ASSERT_THAT(optimizer->activeVertices(), testing::SizeIs(numVertices));
  ASSERT_THAT(optimizer->activeChi2(), testing::Gt(0.));
  ASSERT_TRUE(optimizer->verifyInformationMatrices());

  ASSERT_THAT(
      optimizer->activeVertices(),
      testing::Each(testing::ResultOf(
          [this](g2o::OptimizableGraph::Vertex* v) {
            return optimizer->findActiveVertex(v);
          },
          testing::Not(testing::Eq(optimizer->activeVertices().end())))));

  std::map<int, g2o::Vector3> estimates = vertexEstimates();
  for (size_t i = 1; i < numVertices; ++i)
    ASSERT_DOUBLE_EQ(0, estimates[i].norm());
  optimizer->computeInitialGuess();
  estimates = vertexEstimates();
  for (size_t i = 1; i < numVertices; ++i) ASSERT_LT(0, estimates[i].norm());
}

TEST_F(GeneralGraphOperations, EdgeInit) {
  g2o::OptimizableGraph::Edge* e =
      dynamic_cast<g2o::OptimizableGraph::Edge*>(*optimizer->edges().begin());
  g2o::OptimizableGraph::Edge* e2 = dynamic_cast<g2o::OptimizableGraph::Edge*>(
      *std::next(optimizer->edges().begin()));
  g2o::HyperGraph::EdgeSet eset = {e};

  optimizer->initializeOptimization(eset);

  EXPECT_THAT(optimizer->activeVertices(), testing::SizeIs(2));
  EXPECT_THAT(optimizer->activeEdges(), testing::SizeIs(1));
  EXPECT_THAT(optimizer->findActiveEdge(e),
              testing::Not(testing::Eq(optimizer->activeEdges().end())));
  EXPECT_THAT(optimizer->findActiveEdge(e2),
              testing::Eq(optimizer->activeEdges().end()));
}

TEST_F(GeneralGraphOperations, Gauge) {
  optimizer->initializeOptimization();

  EXPECT_THAT(optimizer->gaugeFreedom(), testing::IsFalse());
  EXPECT_THAT(optimizer->findGauge(), testing::NotNull());

  optimizer->vertex(0)->setFixed(false);
  EXPECT_THAT(optimizer->gaugeFreedom(), testing::IsTrue());
  EXPECT_THAT(optimizer->findGauge(), testing::NotNull());
}

TEST_F(GeneralGraphOperations, Dimensions) {
  optimizer->initializeOptimization();

  EXPECT_EQ(optimizer->maxDimension(), 3);
  EXPECT_THAT(optimizer->dimensions(), testing::ElementsAre(3));

  g2o::VertexPointXY* point = new g2o::VertexPointXY;
  point->setId(numVertices + 1);
  optimizer->addVertex(point);

  EXPECT_EQ(optimizer->maxDimension(), 3);
  EXPECT_THAT(optimizer->dimensions(), testing::ElementsAre(2, 3));
}

TEST_F(GeneralGraphOperations, VerifyInformationMatrices) {
  optimizer->initializeOptimization();

  ASSERT_TRUE(optimizer->verifyInformationMatrices());

  g2o::EdgeSE2* e1 = new g2o::EdgeSE2();
  e1->vertices()[0] = optimizer->vertex(0);
  e1->vertices()[1] = optimizer->vertex(1);
  e1->setMeasurement(g2o::SE2(2., 0., 0.));
  e1->setInformation(-1 * g2o::EdgeSE2::InformationType::Identity());
  optimizer->addEdge(e1);

  g2o::EdgeSE2* e2 = new g2o::EdgeSE2();
  e2->vertices()[0] = optimizer->vertex(1);
  e2->vertices()[1] = optimizer->vertex(2);
  e2->setMeasurement(g2o::SE2(1., 2., 0.));
  g2o::EdgeSE2::InformationType infoMat =
      g2o::EdgeSE2::InformationType::Identity();
  infoMat(1, 2) = 1;
  e2->setInformation(infoMat);
  optimizer->addEdge(e2);

  ASSERT_FALSE(optimizer->verifyInformationMatrices(true));
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

  std::set<int> vertexDims = {2, 3};
  std::set<int> vertexDimsNoMatch = {1, 5};

  EXPECT_TRUE(optimizer->isSolverSuitable(solverPropertyVar));
  EXPECT_TRUE(optimizer->isSolverSuitable(solverPropertyVar, vertexDims));
  EXPECT_TRUE(
      optimizer->isSolverSuitable(solverPropertyVar, vertexDimsNoMatch));

  EXPECT_TRUE(optimizer->isSolverSuitable(solverPropertyFix32));
  EXPECT_TRUE(optimizer->isSolverSuitable(solverPropertyFix32, vertexDims));
  EXPECT_FALSE(
      optimizer->isSolverSuitable(solverPropertyFix32, vertexDimsNoMatch));

  EXPECT_FALSE(optimizer->isSolverSuitable(solverPropertyFix63));
  EXPECT_FALSE(optimizer->isSolverSuitable(solverPropertyFix63, vertexDims));
  EXPECT_FALSE(
      optimizer->isSolverSuitable(solverPropertyFix63, vertexDimsNoMatch));
}

TEST_F(GeneralGraphOperations, JacWorkspace) {
  JacobianWorkspaceTestAdapter workspace;
  ASSERT_THAT(workspace.maxDimension(), testing::Le(0));
  ASSERT_THAT(workspace.maxNumVertices(), testing::Le(0));

  auto* root = optimizer->vertex(0);
  workspace.updateSize(*root->edges().begin(), true);
  EXPECT_THAT(workspace.maxDimension(), testing::Eq(9));
  EXPECT_THAT(workspace.maxNumVertices(), testing::Eq(2));
  EXPECT_THAT(workspace.workspace(), testing::IsEmpty());

  workspace.updateSize(5, 23, true);
  EXPECT_THAT(workspace.maxDimension(), testing::Eq(23));
  EXPECT_THAT(workspace.maxNumVertices(), testing::Eq(5));
  EXPECT_THAT(workspace.workspace(), testing::IsEmpty());

  workspace.updateSize(*optimizer, true);
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
          [](const g2o::VectorX vec) { return vec.isApproxToConstant(0); },
          testing::IsTrue())));
}

namespace {

class TreeVisitor : public g2o::HyperDijkstra::TreeAction {
 public:
  double perform(g2o::HyperGraph::Vertex* v,
                 g2o::HyperGraph::Vertex* /*vParent*/,
                 g2o::HyperGraph::Edge* /*e*/) override {
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
  g2o::HyperDijkstra hyperDijkstra(optimizer.get());
  hyperDijkstra.shortestPaths(optimizer->vertex(0), &uniformCost);

  g2o::HyperDijkstra::computeTree(hyperDijkstra.adjacencyMap());
  TreeVisitor treeVisitor;
  g2o::HyperDijkstra::visitAdjacencyMap(hyperDijkstra.adjacencyMap(),
                                        &treeVisitor);
  EXPECT_THAT(treeVisitor.visitedIds(), testing::SizeIs(numVertices));
  EXPECT_THAT(treeVisitor.visitedIds(),
              testing::UnorderedElementsAreArray(range_helper(numVertices)));
}
