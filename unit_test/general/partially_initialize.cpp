#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "gtest/gtest.h"

TEST(General, JustInitOptimizer) {
  // Initialise optimizer.
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linear_solver_ptr =
      std::make_unique<
          g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  std::unique_ptr<g2o::BlockSolverX> solver_ptr =
      std::make_unique<g2o::BlockSolverX>(std::move(linear_solver_ptr));
  std::unique_ptr<g2o::OptimizationAlgorithmLevenberg> solver =
      std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
          std::move(solver_ptr));
  optimizer.setAlgorithm(solver.release());
  SUCCEED();
}

TEST(General, InsertElementOnlyVertexNoEdge) {
  // Initialise optimizer.
  g2o::SparseOptimizer optimizer;
  std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linear_solver_ptr =
      std::make_unique<
          g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
  std::unique_ptr<g2o::BlockSolverX> solver_ptr =
      std::make_unique<g2o::BlockSolverX>(std::move(linear_solver_ptr));
  std::unique_ptr<g2o::OptimizationAlgorithmLevenberg> solver =
      std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
          std::move(solver_ptr));
  optimizer.setAlgorithm(solver.release());

  const Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

  // Add first vertex.
  g2o::VertexSE3* vertex_0 = new g2o::VertexSE3();
  vertex_0->setId(0);
  vertex_0->setEstimate(transform);
  optimizer.addVertex(vertex_0);

  // Add second vertex.
  g2o::VertexSE3* vertex_1 = new g2o::VertexSE3();
  vertex_1->setId(1);
  vertex_1->setEstimate(transform);
  optimizer.addVertex(vertex_1);

  // Create edge but do not add it to the graph
  g2o::EdgeSE3* edge = new g2o::EdgeSE3();
  edge->setVertex(0, vertex_0);
  edge->setVertex(1, vertex_1);
  edge->setMeasurement(transform);
  SUCCEED();
}
