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

#include "allocate_optimizer.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "gtest/gtest.h"

TEST(General, ClearAndRedo) {
  // Initialize the SparseOptimizer
  std::unique_ptr<g2o::SparseOptimizer> mOptimizerPtr;
  mOptimizerPtr.reset(g2o::internal::createOptimizerForTests());
  g2o::SparseOptimizer& mOptimizer = *mOptimizerPtr;

  // Set the default terminate action
  g2o::SparseOptimizerTerminateAction* terminateAction =
      new g2o::SparseOptimizerTerminateAction;
  mOptimizer.addPostIterationAction(terminateAction);

  for (int i = 0; i < 2; i++) {
    // Add vertices
    g2o::VertexSE3* v0 = new g2o::VertexSE3;
    v0->setEstimate(
        Eigen::Transform<double, 3, 1>(Eigen::Translation<double, 3>(0, 0, 0)));
    v0->setId(0);
    mOptimizer.addVertex(v0);

    g2o::VertexSE3* v1 = new g2o::VertexSE3;
    v1->setEstimate(
        Eigen::Transform<double, 3, 1>(Eigen::Translation<double, 3>(0, 0, 0)));
    v1->setId(1);
    mOptimizer.addVertex(v1);

    g2o::VertexSE3* v2 = new g2o::VertexSE3;
    v2->setEstimate(
        Eigen::Transform<double, 3, 1>(Eigen::Translation<double, 3>(0, 0, 0)));
    v2->setId(2);
    mOptimizer.addVertex(v2);

    // Add edges
    g2o::EdgeSE3* e1 = new g2o::EdgeSE3();
    e1->vertices()[0] = mOptimizer.vertex(0);
    e1->vertices()[1] = mOptimizer.vertex(1);
    e1->setMeasurement(g2o::Isometry3(Eigen::Translation<double, 3>(1, 0, 0)));
    e1->setInformation(g2o::MatrixN<6>::Identity());
    mOptimizer.addEdge(e1);

    g2o::EdgeSE3* e2 = new g2o::EdgeSE3();
    e2->vertices()[0] = mOptimizer.vertex(1);
    e2->vertices()[1] = mOptimizer.vertex(2);
    e2->setMeasurement(g2o::Isometry3(Eigen::Translation<double, 3>(0, 1, 0)));
    e2->setInformation(g2o::MatrixN<6>::Identity());
    mOptimizer.addEdge(e2);

    g2o::EdgeSE3* e3 = new g2o::EdgeSE3();
    e3->vertices()[0] = mOptimizer.vertex(2);
    e3->vertices()[1] = mOptimizer.vertex(0);
    e3->setMeasurement(
        g2o::Isometry3(Eigen::Translation<double, 3>(-0.8, -0.7, 0.1)));
    e3->setInformation(g2o::MatrixN<6>::Identity());
    mOptimizer.addEdge(e3);

    v0->setFixed(true);

    // mOptimizer.setVerbose(true);
    mOptimizer.initializeOptimization();
    mOptimizer.computeInitialGuess();
    mOptimizer.computeActiveErrors();
    int iter = mOptimizer.optimize(10);
    if (iter <= 0) {
      ADD_FAILURE() << "Optimization did not happen";
    } else {
      SUCCEED();
    }

    mOptimizer.clear();
  }
  delete terminateAction;
}
