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

#include <memory>
#include <vector>

#include "g2o/core/block_solver.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "gmock/gmock.h"

namespace {

using namespace testing;  // NOLINT

struct TruePosition {
  int id;
  g2o::Vector2 point;
  TruePosition(int id, const g2o::Vector2& point)  // NOLINT
      : id(id), point(point) {}
};

template <typename T>
void createSolver(g2o::SparseOptimizer& optimizer) {
  auto linearSolver =
      std::make_unique<g2o::LinearSolverEigen<typename T::PoseMatrixType>>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<T>(std::move(linearSolver));
  auto algorithm = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
      std::move(blockSolver));

  optimizer.setAlgorithm(std::move(algorithm));
}

void fillGraph(g2o::SparseOptimizer& graph, bool marginalize_points) {
  srand(42);  // seed Eigen's Random gen

  auto v = std::make_shared<g2o::VertexSE2>();
  v->setId(0);
  v->setEstimate(g2o::SE2());
  v->setFixed(true);
  graph.addVertex(v);

  v = std::make_shared<g2o::VertexSE2>();
  v->setId(1);
  v->setEstimate(g2o::SE2(1.2, 0, 0.1));
  v->setFixed(false);
  graph.addVertex(v);

  // odom edge
  auto e = std::make_shared<g2o::EdgeSE2>();
  e->setInformation(g2o::EdgeSE2::InformationType::Identity());
  e->setMeasurement(g2o::SE2(1.01, 0.02, 0.01));
  e->vertices()[0] = graph.vertex(0);
  e->vertices()[1] = graph.vertex(1);
  graph.addEdge(e);

  // add points
  constexpr int kNumPoints = 5;
  constexpr int kPointIdOffset = 100;
  std::vector<TruePosition> true_point_locations;
  for (int i = 0; i < kNumPoints; ++i) {
    true_point_locations.emplace_back(kPointIdOffset + i,
                                      g2o::Vector2(i * 0.5, i % 2 ? 1. : -1.));

    auto point = std::make_shared<g2o::VertexPointXY>();
    point->setId(true_point_locations.back().id);
    point->setEstimate(true_point_locations.back().point +
                       0.1 * g2o::Vector2::Random());
    point->setMarginalized(marginalize_points);
    graph.addVertex(point);
  }

  const g2o::SE2 true_motion[] = {g2o::SE2(), g2o::SE2(1, 0, 0)};
  for (int k = 0; k < 2; ++k) {
    auto pose = std::static_pointer_cast<g2o::VertexSE2>(graph.vertex(k));
    for (const auto& true_point : true_point_locations) {
      auto point = std::static_pointer_cast<g2o::VertexPointXY>(
          graph.vertex(true_point.id));

      const g2o::Vector2 gt_position =
          true_motion[k].inverse() * true_point.point;

      auto e = std::make_shared<g2o::EdgeSE2PointXY>();
      e->vertices()[0] = pose;
      e->vertices()[1] = point;
      e->setInformation(g2o::EdgeSE2PointXY::InformationType::Identity());
      e->setMeasurement(gt_position + 0.01 * g2o::Vector2::Random());
      graph.addEdge(e);
    }
  }
}

TEST(Slam2DOptimization, CompareSchurError) {
  g2o::SparseOptimizer optimizer;
  g2o::SparseOptimizer optimizer_schur;
  fillGraph(optimizer, false);
  fillGraph(optimizer_schur, true);

  createSolver<g2o::BlockSolverX>(optimizer);
  createSolver<g2o::BlockSolver_3_2>(optimizer_schur);

  auto run = [](g2o::SparseOptimizer& optimizer) {
    constexpr int kNumIterations = 20;
    optimizer.initializeOptimization();
    return optimizer.optimize(kNumIterations);
  };

  int state = run(optimizer);
  int state_schur = run(optimizer_schur);

  ASSERT_THAT(state, Ne(0));
  ASSERT_THAT(state_schur, Ne(0));

  EXPECT_THAT(optimizer.chi2(), DoubleNear(optimizer_schur.chi2(), 1e-5));
}

}  // namespace
