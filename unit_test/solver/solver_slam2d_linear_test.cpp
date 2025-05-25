#include "g2o/solvers/slam2d_linear/solver_slam2d_linear.h"

#include <memory>

#include "g2o/core/block_solver.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gmock/gmock.h"
#include "unit_test/test_helper/eigen_matcher.h"

using namespace ::testing;  // NOLINT

namespace {

std::unique_ptr<g2o::SparseOptimizer> create() {
  using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<3, 2>>;
  using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;

  // Initialize the SparseOptimizer
  auto mOptimizer = std::make_unique<g2o::SparseOptimizer>();
  auto linearSolver = std::make_unique<LinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolver>(std::move(linearSolver));
  mOptimizer->setAlgorithm(std::shared_ptr<g2o::OptimizationAlgorithm>(
      new g2o::SolverSLAM2DLinear(std::move(blockSolver))));
  return mOptimizer;
}

//! Create a perfect circle of nodes/edges
void createCircle(g2o::SparseOptimizer& optimizer, int n, bool fix_root) {
  const double ancle_increment = (2 * M_PI) / (n + 1);
  const g2o::SE2 motion(1, 0, ancle_increment);

  auto root = std::make_shared<g2o::VertexSE2>();
  root->setId(0);
  root->setFixed(fix_root);
  optimizer.addVertex(root);
  std::shared_ptr<g2o::VertexSE2> last_circle_vertex = root;

  for (int i = 0; i < n; ++i) {
    auto next_circle_vertex = std::make_shared<g2o::VertexSE2>();
    next_circle_vertex->setId(i + 1);
    optimizer.addVertex(next_circle_vertex);

    auto e = std::make_shared<g2o::EdgeSE2>();
    e->setVertex(0, last_circle_vertex);
    e->setVertex(1, next_circle_vertex);
    e->setMeasurement(motion);
    e->setInformation(g2o::EdgeSE2::InformationType::Identity());
    optimizer.addEdge(e);
    last_circle_vertex = next_circle_vertex;
  }

  // Connect last and root
  auto loop_closing = std::make_shared<g2o::EdgeSE2>();
  loop_closing->setVertex(0, last_circle_vertex);
  loop_closing->setVertex(1, root);
  loop_closing->setMeasurement(motion);
  loop_closing->setInformation(g2o::EdgeSE2::InformationType::Identity());
  optimizer.addEdge(loop_closing);
}

TEST(SolverSLAM2DLinear, SolveCircleNoFixed) {
  std::unique_ptr<g2o::SparseOptimizer> optimizer = create();
  createCircle(*optimizer, 10, false);

  optimizer->initializeOptimization();
  const int iter_preformed = optimizer->optimize(5);
  EXPECT_THAT(iter_preformed, Eq(0));
}

TEST(SolverSLAM2DLinear, SolveCircleNotAllVertices) {
  constexpr int kNumVertices = 10;
  std::unique_ptr<g2o::SparseOptimizer> optimizer = create();
  createCircle(*optimizer, kNumVertices, true);

  g2o::HyperGraph::VertexSet vset;
  for (const auto& id_v : optimizer->vertices()) {
    if (id_v.first > kNumVertices - 3) continue;
    vset.insert(id_v.second);
  }

  optimizer->initializeOptimization(vset);
  const int iter_preformed = optimizer->optimize(5);
  EXPECT_THAT(iter_preformed, Eq(0));
}

TEST(SolverSLAM2DLinear, SolveCircle) {
  constexpr int kNumVertices = 10;
  std::unique_ptr<g2o::SparseOptimizer> optimizer = create();
  createCircle(*optimizer, kNumVertices, true);

  optimizer->initializeOptimization();
  const int iter_preformed = optimizer->optimize(5);
  EXPECT_THAT(iter_preformed, Gt(0));

  for (const auto& id_v : optimizer->vertices()) {
    const g2o::VertexSE2* v = static_cast<g2o::VertexSE2*>(id_v.second.get());
    if (id_v.first == 0) {
      EXPECT_THAT(v->estimate().toVector(),
                  g2o::internal::EigenApproxEqual(g2o::Vector3::Zero(), 1e-6));
      continue;
    }
    EXPECT_THAT(v->estimate().rotation().angle(), Ne(0.));
  }
}

}  // namespace
