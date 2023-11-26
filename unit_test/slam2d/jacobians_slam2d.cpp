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

#include <gtest/gtest.h>

#include "g2o/stuff/sampler.h"
#include "g2o/types/slam2d/edge_pointxy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2_pointxy_bearing.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "unit_test/test_helper/evaluate_jacobian.h"

using namespace g2o;  // NOLINT

namespace {
SE2 randomSE2() { return SE2(Vector3::Random()); }
}  // namespace

TEST(Slam2D, EdgeSE2Jacobian) {
  auto v1 = std::make_shared<VertexSE2>();
  v1->setId(0);

  auto v2 = std::make_shared<VertexSE2>();
  v2->setId(1);

  EdgeSE2 e;
  e.setVertex(0, v1);
  e.setVertex(1, v2);
  e.setInformation(EdgeSE2::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1->setEstimate(randomSE2());
    v2->setEstimate(randomSE2());
    e.setMeasurement(randomSE2());

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}

TEST(Slam2D, EdgeSE2PriorJacobian) {
  auto v1 = std::make_shared<VertexSE2>();
  v1->setId(0);

  EdgeSE2Prior e;
  e.setVertex(0, v1);
  e.setInformation(EdgeSE2::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1->setEstimate(randomSE2());
    e.setMeasurement(randomSE2());

    evaluateJacobianUnary(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}

TEST(Slam2D, EdgePointXYJacobian) {
  auto v1 = std::make_shared<VertexPointXY>();
  v1->setId(0);

  auto v2 = std::make_shared<VertexPointXY>();
  v2->setId(1);

  EdgePointXY e;
  e.setVertex(0, v1);
  e.setVertex(1, v2);
  e.setInformation(EdgePointXY::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1->setEstimate(Eigen::Vector2d::Random());
    v2->setEstimate(Eigen::Vector2d::Random());
    e.setMeasurement(Eigen::Vector2d::Random());

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}

TEST(Slam2D, EdgeSE2PointXYJacobian) {
  auto v1 = std::make_shared<VertexSE2>();
  v1->setId(0);

  auto v2 = std::make_shared<VertexPointXY>();
  v2->setId(1);

  EdgeSE2PointXY e;
  e.setVertex(0, v1);
  e.setVertex(1, v2);
  e.setInformation(EdgeSE2PointXY::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1->setEstimate(randomSE2());
    v2->setEstimate(Eigen::Vector2d::Random());
    e.setMeasurement(Eigen::Vector2d::Random());

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}

TEST(Slam2D, EdgeSE2PointXYBearingJacobian) {
  auto v1 = std::make_shared<VertexSE2>();
  v1->setId(0);

  auto v2 = std::make_shared<VertexPointXY>();
  v2->setId(1);

  EdgeSE2PointXYBearing e;
  e.setVertex(0, v1);
  e.setVertex(1, v2);
  e.setInformation(EdgeSE2PointXYBearing::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1->setEstimate(randomSE2());
    v2->setEstimate(Eigen::Vector2d::Random());
    e.setMeasurement(g2o::Sampler::uniformRand(0., 1.) * M_PI);

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}
