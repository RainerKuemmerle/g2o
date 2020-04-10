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

#include "unit_test/test_helper/evaluate_jacobian.h"

#include "g2o/core/jacobian_workspace.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

#include "g2o/core/optimizable_graph.h"

using namespace std;
using namespace g2o;
using namespace g2o::internal;

namespace {

static Eigen::Isometry3d randomIsometry3d()
{
  Eigen::Vector3d rotAxisAngle = Eigen::Vector3d::Random();
  rotAxisAngle += Eigen::Vector3d::Random();
  Eigen::AngleAxisd rotation(rotAxisAngle.norm(), rotAxisAngle.normalized());
  Eigen::Isometry3d result = (Eigen::Isometry3d)rotation.toRotationMatrix();
  result.translation() = Eigen::Vector3d::Random();
  return result;
}

static Sim3 randomSim3()
{
  Eigen::Isometry3d se3 = randomIsometry3d();
  Eigen::Matrix3d r = se3.rotation();
  Eigen::Vector3d t = se3.translation();
  return Sim3(r, t, 1.0);
}

} // end anonymous namespace

TEST(Sim3, EdgeSim3Jacobian)
{
  VertexSim3Expmap v1;
  v1.setId(0); 

  VertexSim3Expmap v2;
  v2.setId(1); 

  EdgeSim3 e;
  e.setVertex(0, &v1);
  e.setVertex(1, &v2);
  e.setInformation(EdgeSim3::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 1; ++k) {
    v1.setEstimate(randomSim3());
    v2.setEstimate(randomSim3());
    e.setMeasurement(randomSim3());

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}