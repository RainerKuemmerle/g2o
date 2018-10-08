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

#include "gtest/gtest.h"

#include "g2o/types/slam3d/edge_se3.h"
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

using namespace std;
using namespace g2o;

typedef g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType> SlamLinearSolver;

TEST(Slam3D, OptimizationEdgeSE3Translation)
{
  g2o::SparseOptimizer optimizer;
  
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
  g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  optimizer.setAlgorithm(algorithm);

  g2o::VertexSE3 *v = new g2o::VertexSE3();
  v->setId(0);
  v->setEstimate(g2o::Isometry3::Identity());
  v->setFixed(true);
  optimizer.addVertex(v);

  v = new g2o::VertexSE3();
  v->setId(1);
  // move vertex away from origin
  g2o::Isometry3 p2 = g2o::Isometry3::Identity();
  p2.translation() << 10., 10., 10.;
  v->setEstimate(p2);
  v->setFixed(false);
  optimizer.addVertex(v);

  g2o::EdgeSE3 *e = new g2o::EdgeSE3();
  e->setInformation(g2o::EdgeSE3::InformationType::Identity());
  e->setMeasurement(g2o::Isometry3::Identity());
  e->vertices()[0] = optimizer.vertex(0);
  e->vertices()[1] = optimizer.vertex(1);
  optimizer.addEdge(e);

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  ASSERT_LT(0., optimizer.chi2());
  int numOptimization = optimizer.optimize(100);
  ASSERT_LT(0, numOptimization);
  ASSERT_GT(1e-6, optimizer.chi2());

  g2o::VertexSE3 *v2AfterOpti = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(1));
  ASSERT_DOUBLE_EQ(0., (v2AfterOpti->estimate().translation() - g2o::Vector3::Zero()).norm());
  ASSERT_DOUBLE_EQ(0., (v2AfterOpti->estimate().rotation().diagonal() - g2o::Vector3::Ones()).norm());
}

TEST(Slam3D, OptimizationEdgeSE3Rotation)
{
  g2o::SparseOptimizer optimizer;
  
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
  g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  optimizer.setAlgorithm(algorithm);

  g2o::VertexSE3 *v = new g2o::VertexSE3();
  v->setId(0);
  v->setEstimate(g2o::Isometry3::Identity());
  v->setFixed(true);
  optimizer.addVertex(v);

  v = new g2o::VertexSE3();
  v->setId(1);
  // rotate vertex a bit
  g2o::Isometry3 p2 = g2o::Isometry3::Identity();
  p2 *= g2o::AngleAxis(g2o::deg2rad(2), g2o::Vector3::Ones().normalized());
  v->setEstimate(p2);
  v->setFixed(false);
  optimizer.addVertex(v);

  g2o::EdgeSE3 *e = new g2o::EdgeSE3();
  e->setInformation(g2o::EdgeSE3::InformationType::Identity());
  e->setMeasurement(g2o::Isometry3::Identity());
  e->vertices()[0] = optimizer.vertex(0);
  e->vertices()[1] = optimizer.vertex(1);
  optimizer.addEdge(e);

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  ASSERT_LT(0., optimizer.chi2());
  int numOptimization = optimizer.optimize(100);
  ASSERT_LT(0, numOptimization);
  ASSERT_GT(1e-6, optimizer.chi2());

  g2o::VertexSE3 *v2AfterOpti = dynamic_cast<g2o::VertexSE3*>(optimizer.vertex(1));
  ASSERT_DOUBLE_EQ(0., (v2AfterOpti->estimate().translation() - g2o::Vector3::Zero()).norm());
  ASSERT_DOUBLE_EQ(0., (v2AfterOpti->estimate().rotation().diagonal() - g2o::Vector3::Ones()).norm());
}