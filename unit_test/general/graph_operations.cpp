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

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/stuff/string_tools.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);

static g2o::SparseOptimizer* createOptimizer() {
  // Initialize the SparseOptimizer
  g2o::SparseOptimizer* mOptimizer = new g2o::SparseOptimizer();
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = g2o::make_unique<SlamBlockSolver>(std::move(linearSolver));
  mOptimizer->setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver)));
  return mOptimizer;
}

TEST(General, BinaryEdgeConstructor) {
  g2o::EdgeSE3 e1;
  ASSERT_EQ(NULL, e1.vertices()[0]);
  ASSERT_EQ(NULL, e1.vertices()[1]);

  g2o::EdgeSE2 e2;
  ASSERT_EQ(NULL, e2.vertices()[0]);
  ASSERT_EQ(NULL, e2.vertices()[1]);
}

TEST(General, GraphAddVertex) {
  g2o::SparseOptimizer* optimizer = createOptimizer();

  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_EQ(optimizer, v1->graph());
  ASSERT_EQ(size_t(1), optimizer->vertices().size());
  ASSERT_FALSE(optimizer->addVertex(v1));
  ASSERT_EQ(size_t(1), optimizer->vertices().size());

  {
    g2o::VertexSE2* v2 = new g2o::VertexSE2();
    v2->setId(0);
    ASSERT_FALSE(optimizer->addVertex(v2));
    ASSERT_EQ(size_t(1), optimizer->vertices().size());
    ASSERT_EQ(NULL, v2->graph());
    delete v2;
  }

  delete optimizer;
}

TEST(General, GraphAddEdge) {
  g2o::SparseOptimizer* optimizer = createOptimizer();

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
  ASSERT_FALSE(optimizer->addEdge(e2)) << "Adding edge with unset vertices was possible";
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(NULL, e2->graph());

  g2o::EdgeSE2* e3 = new g2o::EdgeSE2();
  e3->setVertex(0, v1);
  e3->setVertex(1, v1);
  ASSERT_FALSE(optimizer->addEdge(e3)) << "Adding binary edge with same vertices was possible";

  delete optimizer;
}

/**
 * Fixture to test saving and loading of a graph.
 * Here, we will have a simple graph with N nodes and N edges.
 * We use for saving and loading.
 */
class GeneralGraphLoadSave : public ::testing::Test {
 protected:
  void SetUp() override {
    optimizer.reset(createOptimizer());

    // Add vertices
    for (int i = 0; i < numVertices; ++i) {
      g2o::VertexSE2* v = new g2o::VertexSE2;
      v->setEstimate(g2o::SE2());
      v->setId(i);
      v->setFixed(i == 0);  // fix the first vertex
      optimizer->addVertex(v);
    }

    // Add edges
    for (int i = 0; i < numVertices; ++i) {
      g2o::EdgeSE2* e1 = new g2o::EdgeSE2();
      e1->vertices()[0] = optimizer->vertex((i + 0) % numVertices);
      e1->vertices()[1] = optimizer->vertex((i + 1) % numVertices);
      e1->setMeasurement(g2o::SE2());
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

  //! recover the vertec IDs of the edges from a output generated by OptimizableGraph::save()
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
      result.emplace_back(std::make_pair((i + 0) % numVertices, (i + 1) % numVertices));
    return result;
  }

  std::unique_ptr<g2o::SparseOptimizer> optimizer;
  size_t numVertices = 3;
};

TEST_F(GeneralGraphLoadSave, SavingGraph) {
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

TEST_F(GeneralGraphLoadSave, LoadingGraph) {
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

  ASSERT_THAT(optimizer->vertices(),
              testing::UnorderedElementsAreArray(internal::VectorIntToKeys(expectedIds())));
  ASSERT_THAT(optimizer->edges(), testing::Each(testing::Property(
                                      &g2o::OptimizableGraph::Edge::vertices, testing::SizeIs(2))));
  ASSERT_THAT(optimizer->edges(), testing::Each(testing::ResultOf(
                                      [](g2o::HyperGraph::Edge* e) {
                                        return std::make_pair(e->vertex(0)->id(),
                                                              e->vertex(1)->id());
                                      },
                                      testing::AnyOfArray(expectedEdgeIds()))));
}
