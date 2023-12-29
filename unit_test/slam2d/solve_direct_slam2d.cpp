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

#include "g2o/core/eigen_types.h"
#include "g2o/core/optimization_algorithm_with_hessian.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gmock/gmock.h"
#include "unit_test/test_helper/allocate_optimizer.h"
#include "unit_test/test_helper/eigen_matcher.h"

using namespace g2o;      // NOLINT
using namespace testing;  // NOLINT
using g2o::internal::print_wrap;

TEST(Slam2D, SolveDirect) {
  std::unique_ptr<g2o::SparseOptimizer> optimizer =
      g2o::internal::createOptimizerForTests();

  OptimizationAlgorithmWithHessian* solverWithHessian =
      dynamic_cast<g2o::OptimizationAlgorithmWithHessian*>(
          optimizer->solver().get());
  ASSERT_THAT(solverWithHessian, Ne(nullptr));

  auto v1 = std::make_shared<VertexSE2>();
  v1->setId(0);
  ASSERT_THAT(optimizer->addVertex(v1), IsTrue());

  auto v2 = std::make_shared<VertexPointXY>();
  v2->setEstimate(Vector2(1, 0.5));
  v2->setId(1);
  ASSERT_THAT(optimizer->addVertex(v2), IsTrue());

  auto e = std::make_shared<g2o::EdgeSE2PointXY>();
  e->setVertex(0, v1);
  e->setVertex(1, v2);
  e->setMeasurement(EdgeSE2PointXY::Measurement(2, 1));
  e->setInformation(EdgeSE2PointXY::InformationType::Identity());
  ASSERT_THAT(optimizer->addEdge(e), IsTrue());

  HyperGraph::EdgeSet edges_to_optimize = {e};

  optimizer->initializeOptimization(edges_to_optimize);
  optimizer->computeInitialGuess();
  optimizer->solver()->init();
  ASSERT_THAT(solverWithHessian->buildLinearStructure(), IsTrue())
      << "FATAL: failure while building linear structure";
  optimizer->computeActiveErrors();
  solverWithHessian->updateLinearSystem();

  const bool solve_status = v2->solveDirect();
  EXPECT_THAT(solve_status, IsTrue());
  EXPECT_THAT(print_wrap(v2->estimate()),
              EigenApproxEqual(print_wrap(Vector2(2, 1)), 1e-3));
}
