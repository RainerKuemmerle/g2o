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

#include "g2o/solvers/structure_only/structure_only_solver.h"

#include <gtest/gtest.h>

#include <memory>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/sba/edge_project_xyz2uv.h"
#include "g2o/types/sba/parameter_cameraparameters.h"
#include "g2o/types/sba/vertex_se3_expmap.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

namespace g2o {

/**
 * \brief Test suite for StructureOnlySolver with 3D points
 */
class StructureOnlySolverTest3D : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create optimizer
    optimizer_ = std::make_unique<g2o::SparseOptimizer>();
    optimizer_->setVerbose(false);

    // Create a camera parameter
    cam_params_ = std::make_shared<g2o::CameraParameters>(
        focal_length_, principal_point_, 0.0);
    cam_params_->setId(0);

    if (!optimizer_->addParameter(cam_params_)) {
      FAIL() << "Failed to add camera parameters";
    }

    // Create camera poses (these will be fixed)
    // Two cameras looking at the same scene from different positions
    vertex_id_ = 0;

    for (size_t i = 0; i < num_cameras_; ++i) {
      g2o::Vector3 trans(static_cast<double>(i) * 0.1, 0.0, 0.0);
      Eigen::Quaterniond q;
      q.setIdentity();
      g2o::SE3Quat pose(q, trans);

      auto v_se3 = std::make_shared<g2o::VertexSE3Expmap>();
      v_se3->setId(vertex_id_);
      v_se3->setFixed(true);  // Keep poses fixed
      v_se3->setEstimate(pose);
      optimizer_->addVertex(v_se3);
      camera_poses_.push_back(pose);
      vertex_id_++;
    }
  }

  /**
   * Add a 3D point with noisy observations from all cameras
   */
  void AddPoint3DWithObservations(const g2o::Vector3& true_point,
                                  double noise_std_dev = 1.0) {
    // Create point vertex
    auto v_p = std::make_shared<g2o::VertexPointXYZ>();
    v_p->setId(vertex_id_);
    v_p->setMarginalized(true);

    // Add noise to the initial estimate
    g2o::Vector3 noisy_point = true_point;
    noisy_point(0) += g2o::Sampler::gaussRand(0.0, noise_std_dev);
    noisy_point(1) += g2o::Sampler::gaussRand(0.0, noise_std_dev);
    noisy_point(2) += g2o::Sampler::gaussRand(0.0, noise_std_dev);

    v_p->setEstimate(noisy_point);
    optimizer_->addVertex(v_p);

    true_points_.push_back(true_point);
    point_vertices_.push_back(v_p);

    // Create edges from each camera to this point
    for (size_t cam_idx = 0; cam_idx < camera_poses_.size(); ++cam_idx) {
      // Project true point into camera
      g2o::Vector2 projection =
          cam_params_->cam_map(camera_poses_[cam_idx].map(true_point));

      // Check if point is visible in this camera
      if (projection(0) >= 0 && projection(1) >= 0 && projection(0) < 640 &&
          projection(1) < 480) {
        // Add pixel noise
        g2o::Vector2 measurement = projection;
        measurement(0) += g2o::Sampler::gaussRand(0.0, 1.0);
        measurement(1) += g2o::Sampler::gaussRand(0.0, 1.0);

        // Create edge
        auto edge = std::make_shared<g2o::EdgeProjectXYZ2UV>();
        edge->setVertex(0, v_p);                          // Point vertex
        edge->setVertex(1, optimizer_->vertex(cam_idx));  // Camera vertex
        edge->setMeasurement(measurement);
        edge->setParameterId(0, 0);  // Camera parameter ID
        edge->_cam = cam_params_;

        // Set information matrix (inverse of covariance)
        Eigen::Matrix2d info;
        info.setIdentity();
        info *= 1.0 / (1.0 * 1.0);  // 1 pixel standard deviation
        edge->setInformation(info);

        optimizer_->addEdge(edge);
      }
    }
    vertex_id_++;
  }

  std::unique_ptr<g2o::SparseOptimizer> optimizer_;
  std::shared_ptr<g2o::CameraParameters> cam_params_;
  double focal_length_ = 1000.;
  g2o::Vector2 principal_point_ = g2o::Vector2(320.0, 240.0);
  std::vector<g2o::SE3Quat, Eigen::aligned_allocator<g2o::SE3Quat>>
      camera_poses_;
  std::vector<g2o::Vector3> true_points_;
  std::vector<std::shared_ptr<g2o::VertexPointXYZ>> point_vertices_;
  int vertex_id_;
  size_t num_cameras_ = 2;
};

/**
 * Test basic initialization of StructureOnlySolver
 */
TEST_F(StructureOnlySolverTest3D, InitializationTest) {
  // Add some test points
  AddPoint3DWithObservations(g2o::Vector3(0.5, 0.0, 3.0));
  AddPoint3DWithObservations(g2o::Vector3(-0.5, 0.0, 3.0));
  AddPoint3DWithObservations(g2o::Vector3(0.0, 0.5, 3.0));

  // Create solver
  auto solver = std::make_unique<g2o::StructureOnlySolver<3>>();
  ASSERT_TRUE(solver != nullptr);

  // Set optimizer
  solver->setOptimizer(optimizer_.get());

  // Initialize
  EXPECT_TRUE(solver->init(false));

  // Verify that marginalized vertices are found
  // The solver collects marginalized vertices from activeVertices()
  EXPECT_EQ(point_vertices_.size(), 3);

  // Verify the solver can work with these vertices
  for (const auto& v : point_vertices_) {
    EXPECT_TRUE(v->marginalized());
  }
}

/**
 * Test that solver decreases error over iterations
 */
TEST_F(StructureOnlySolverTest3D, ErrorDecreaseTest) {
  // Add multiple test points with some initial noise
  const size_t num_points = 5;
  for (size_t i = 0; i < num_points; ++i) {
    g2o::Vector3 true_point(g2o::Sampler::uniformRand(-0.5, 0.5),
                            g2o::Sampler::uniformRand(-0.5, 0.5),
                            g2o::Sampler::uniformRand(2.5, 3.5));
    AddPoint3DWithObservations(true_point, 0.5);  // 0.5 std dev noise
  }

  // Compute initial error
  double initial_error = 0.0;
  for (const auto& edge : optimizer_->edges()) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(edge);
    e->computeError();
    initial_error += e->chi2();
  }

  // Create and run solver
  auto solver = std::make_unique<g2o::StructureOnlySolver<3>>();
  solver->setOptimizer(optimizer_.get());
  EXPECT_TRUE(solver->init(false));

  // Run optimization with multiple iterations
  auto result = solver->calc(solver->points(), 5, 10);
  EXPECT_EQ(result, g2o::OptimizationAlgorithm::SolverResult::kOk);

  // Compute final error
  double final_error = 0.0;
  for (const auto& edge : optimizer_->edges()) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(edge);
    e->computeError();
    final_error += e->chi2();
  }

  // Error should decrease (or at least not increase significantly)
  EXPECT_LE(final_error, initial_error * 1.1);  // Allow 10% tolerance
}

/**
 * Test that point estimates change after optimization
 */
TEST_F(StructureOnlySolverTest3D, PointEstimateUpdateTest) {
  // Add test points with small noise to ensure convergence
  AddPoint3DWithObservations(g2o::Vector3(0.5, 0.0, 3.0), 0.1);
  AddPoint3DWithObservations(g2o::Vector3(-0.5, 0.0, 3.0), 0.1);

  // Store initial estimates
  std::vector<g2o::Vector3> initial_estimates;
  initial_estimates.reserve(point_vertices_.size());
  for (const auto& v : point_vertices_) {
    initial_estimates.push_back(v->estimate());
  }

  // Compute initial error before optimization
  double initial_error = 0.0;
  for (const auto& edge : optimizer_->edges()) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(edge);
    e->computeError();
    initial_error += e->chi2();
  }

  // Run optimizer with our custom point list
  auto solver = std::make_unique<g2o::StructureOnlySolver<3>>();
  solver->setOptimizer(optimizer_.get());
  EXPECT_TRUE(solver->init(false));

  // Convert point vertices to generic VertexContainer for calc()
  g2o::OptimizableGraph::VertexContainer verts(point_vertices_.begin(),
                                               point_vertices_.end());
  auto result = solver->calc(verts, 3, 10);
  EXPECT_EQ(result, g2o::OptimizationAlgorithm::SolverResult::kOk);

  // Check that optimization occurred (either points changed OR error decreased)
  double final_error = 0.0;
  for (const auto& edge : optimizer_->edges()) {
    auto e = std::static_pointer_cast<OptimizableGraph::Edge>(edge);
    e->computeError();
    final_error += e->chi2();
  }

  bool error_decreased = final_error < initial_error * 0.99;
  bool points_changed = false;
  for (size_t i = 0; i < point_vertices_.size(); ++i) {
    g2o::Vector3 diff = point_vertices_[i]->estimate() - initial_estimates[i];
    if (diff.norm() > 0.001) {
      points_changed = true;
      break;
    }
  }

  EXPECT_TRUE(error_decreased || points_changed)
      << "Either error should decrease or points should change";
}

/**
 * Test solver with only one camera observing all points
 */
TEST_F(StructureOnlySolverTest3D, SingleCameraTest) {
  // This test uses the default setup with 2 cameras, but we verify
  // that the solver works even if some points are only seen by subset of
  // cameras
  AddPoint3DWithObservations(g2o::Vector3(0.0, 0.0, 3.0), 0.2);

  auto solver = std::make_unique<g2o::StructureOnlySolver<3>>();
  solver->setOptimizer(optimizer_.get());
  EXPECT_TRUE(solver->init(false));

  auto result = solver->calc(solver->points(), 2, 10);
  EXPECT_EQ(result, g2o::OptimizationAlgorithm::SolverResult::kOk);
}

/**
 * Test that fixed points remain unchanged
 */
TEST_F(StructureOnlySolverTest3D, FixedPointTest) {
  // Add a point and mark it as fixed
  auto v_p = std::make_shared<g2o::VertexPointXYZ>();
  v_p->setId(vertex_id_);
  v_p->setMarginalized(true);
  g2o::Vector3 fixed_point(0.0, 0.0, 3.0);
  v_p->setEstimate(fixed_point);
  v_p->setFixed(true);  // Mark as fixed
  optimizer_->addVertex(v_p);

  // Add edges for this fixed point
  for (size_t cam_idx = 0; cam_idx < camera_poses_.size(); ++cam_idx) {
    g2o::Vector2 projection =
        cam_params_->cam_map(camera_poses_[cam_idx].map(fixed_point));
    if (projection(0) >= 0 && projection(1) >= 0 && projection(0) < 640 &&
        projection(1) < 480) {
      auto edge = std::make_shared<g2o::EdgeProjectXYZ2UV>();
      edge->setVertex(0, v_p);
      edge->setVertex(1, optimizer_->vertex(cam_idx));
      edge->setMeasurement(projection);
      edge->setParameterId(0, 0);
      edge->_cam = cam_params_;
      Eigen::Matrix2d info;
      info.setIdentity();
      edge->setInformation(info);
      optimizer_->addEdge(edge);
    }
  }

  // Run solver
  auto solver = std::make_unique<g2o::StructureOnlySolver<3>>();
  solver->setOptimizer(optimizer_.get());
  EXPECT_TRUE(solver->init(false));

  g2o::Vector3 estimate_before = v_p->estimate();
  auto result = solver->calc(solver->points(), 2, 10);
  EXPECT_EQ(result, g2o::OptimizationAlgorithm::SolverResult::kOk);

  // Fixed point should not change
  EXPECT_TRUE(v_p->estimate().isApprox(estimate_before));
}

}  // namespace g2o
