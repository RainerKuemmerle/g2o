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

#include <numeric>

#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/allocate_optimizer.h"

G2O_USE_TYPE_GROUP(slam2d);

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

  {
    auto v2 = std::make_shared<g2o::VertexSE2>();
    v2->setId(0);
    ASSERT_FALSE(optimizer->addVertex(v2));
    ASSERT_EQ(size_t(1), optimizer->vertices().size());
    ASSERT_EQ(nullptr, v2->graph());
  }

  // removing vertex
  ASSERT_TRUE(optimizer->removeVertex(v1));
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

/**
 * Fixture to test saving and loading of a graph.
 * Here, we will have a simple graph with N nodes and N edges.
 * We use for saving and loading.
 */
class GeneralGraphOperations : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer = g2o::internal::createOptimizerForTests();

    // Add vertices
    for (size_t i = 0; i < numVertices; ++i) {
      auto v = std::make_shared<g2o::VertexSE2>();
      v->setEstimate(g2o::SE2());
      v->setId(i);
      v->setFixed(i == 0);  // fix the first vertex
      optimizer->addVertex(v);
    }

    // Add edges
    for (size_t i = 0; i < numVertices; ++i) {
      auto e1 = std::make_shared<g2o::EdgeSE2>();
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
      result.emplace_back(
          std::make_pair((i + 0) % numVertices, (i + 1) % numVertices));
    return result;
  }

  std::map<int, g2o::Vector3> vertexEstimates() const {
    std::map<int, g2o::Vector3> result;
    for (const auto& idV : optimizer->vertices()) {
      g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
      result[idV.first] = v->estimate().toVector();
    }
    return result;
  }

  //! returns the expected Vertex IDs of the fixture
  std::vector<int> fixedIds() const {
    std::vector<int> result;
    for (const auto& idV : optimizer->vertices()) {
      g2o::OptimizableGraph::Vertex* v =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(idV.second.get());
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
  ASSERT_THAT(
      optimizer->edges(),
      testing::Each(testing::Pointee(testing::Property(
          &g2o::OptimizableGraph::Edge::vertices, testing::SizeIs(2)))));
  ASSERT_THAT(
      optimizer->edges(),
      testing::Each(testing::ResultOf(
          [](const std::shared_ptr<g2o::HyperGraph::Edge>& e) {
            return static_cast<g2o::OptimizableGraph::Edge*>(e.get())->graph();
          },
          testing::Eq(static_cast<g2o::OptimizableGraph*>(optimizer.get())))));
  ASSERT_THAT(optimizer->edges(),
              testing::Each(testing::ResultOf(
                  [](const std::shared_ptr<g2o::HyperGraph::Edge>& e) {
                    return std::make_pair(e->vertex(0)->id(),
                                          e->vertex(1)->id());
                  },
                  testing::AnyOfArray(expectedEdgeIds()))));
}

TEST_F(GeneralGraphOperations, PushPopActiveVertices) {
  optimizer->initializeOptimization();
  std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer->push();
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
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
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushDiscardActiveVertices) {
  optimizer->initializeOptimization();
  std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer->push();
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
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
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushPopOptimizableGraph) {
  std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer->OptimizableGraph::push();
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
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
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
    ASSERT_THAT(v->stackSize(), testing::Eq(0));
  }
}

TEST_F(GeneralGraphOperations, PushDiscardOptimizableGraph) {
  optimizer->initializeOptimization();
  std::map<int, g2o::Vector3> originalEstimates = vertexEstimates();

  // push and apply some transformation
  optimizer->OptimizableGraph::push();
  for (const auto& idV : optimizer->vertices()) {
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
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
    g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
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
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(elem.get());
      ASSERT_THAT(v->stackSize(), testing::Eq(1));
    }
    for (const auto& elem : notPushedSet) {
      g2o::OptimizableGraph::Vertex* v =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(elem.get());
      ASSERT_THAT(v->stackSize(), testing::Eq(0));
    }
    // pop or discard
    if (operation == 0)
      optimizer->pop(pushingSet);
    else
      optimizer->discardTop(pushingSet);
    for (const auto& idV : optimizer->vertices()) {
      g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(idV.second.get());
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
          [this](const std::shared_ptr<g2o::OptimizableGraph::Vertex>& v) {
            return optimizer->findActiveVertex(v.get());
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
  auto e = std::dynamic_pointer_cast<g2o::OptimizableGraph::Edge>(
      *optimizer->edges().begin());
  auto e2 = std::dynamic_pointer_cast<g2o::OptimizableGraph::Edge>(
      *std::next(optimizer->edges().begin()));
  g2o::HyperGraph::EdgeSet eset = {e};

  optimizer->initializeOptimization(eset);

  EXPECT_THAT(optimizer->activeVertices(), testing::SizeIs(2));
  EXPECT_THAT(optimizer->activeEdges(), testing::SizeIs(1));
  EXPECT_THAT(optimizer->findActiveEdge(e.get()),
              testing::Not(testing::Eq(optimizer->activeEdges().end())));
  EXPECT_THAT(optimizer->findActiveEdge(e2.get()),
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

  auto point = std::make_shared<g2o::VertexPointXY>();
  point->setId(numVertices + 1);
  optimizer->addVertex(point);

  EXPECT_EQ(optimizer->maxDimension(), 3);
  EXPECT_THAT(optimizer->dimensions(), testing::ElementsAre(2, 3));
}

TEST_F(GeneralGraphOperations, VerifyInformationMatrices) {
  optimizer->initializeOptimization();

  ASSERT_TRUE(optimizer->verifyInformationMatrices());

  auto e1 = std::make_shared<g2o::EdgeSE2>();
  e1->vertices()[0] = optimizer->vertex(0);
  e1->vertices()[1] = optimizer->vertex(1);
  e1->setMeasurement(g2o::SE2(2., 0., 0.));
  e1->setInformation(-1 * g2o::EdgeSE2::InformationType::Identity());
  optimizer->addEdge(e1);

  auto e2 = std::make_shared<g2o::EdgeSE2>();
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

TEST_F(GeneralGraphOperations, SharedOwnerShip) {
  using Ptr = std::shared_ptr<g2o::HyperGraph::Vertex>;
  using IdVertexPair = std::pair<int, Ptr>;

  g2o::HyperGraph::VertexIDMap verticesMap = optimizer->vertices();
  optimizer->clear();

  // currently vertices are owned by our array and the edges
  ASSERT_THAT(
      verticesMap,
      testing::Each(Field(&IdVertexPair::second,
                          testing::Property(&Ptr::use_count, testing::Ge(1)))));

  // clear all edges of each vertex
  for (auto it = verticesMap.begin(); it != verticesMap.end(); ++it)
    it->second->edges().clear();
  ASSERT_THAT(
      verticesMap,
      testing::Each(Field(&IdVertexPair::second,
                          testing::Property(&Ptr::use_count, testing::Ge(1)))));

  Ptr singleVertex = verticesMap.begin()->second;
  verticesMap.clear();
  ASSERT_THAT(singleVertex.use_count(), testing::Eq(1));
}
