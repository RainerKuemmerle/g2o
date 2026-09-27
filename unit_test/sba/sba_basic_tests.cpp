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
#include "unit_test/test_helper/random_state.h"
#include "unit_test/test_helper/typed_basic_tests.h"

#include "g2o/core/factory.h"
#include "g2o/types/sba/edge_project_p2mc.h"
#include "g2o/types/sba/edge_project_p2sc.h"
#include "g2o/types/sba/edge_project_stereo_xyz.h"
#include "g2o/types/sba/edge_project_stereo_xyz_onlypose.h"
#include "g2o/types/sba/edge_project_xyz.h"
#include "g2o/types/sba/edge_project_xyz_onlypose.h"
#include "g2o/types/sba/edge_sba_cam.h"
#include "g2o/types/sba/edge_sba_scale.h"
#include "g2o/types/sba/edge_se3_expmap.h"

G2O_USE_TYPE_GROUP(slam3d)

namespace {
template <int VertexIndex, typename EdgeType>
void fillNumericJacobian(EdgeType& edge, g2o::JacobianWorkspace& workspace,
                         double step) {
  auto vertex = edge.template vertexXn<VertexIndex>();
  if (vertex->fixed()) return;

  using VertexType = typename EdgeType::template VertexXnType<VertexIndex>;
  using JacobianMap =
      typename EdgeType::template JacobianType<EdgeType::kDimension,
                                               VertexType::kDimension>;
  JacobianMap jacobian(workspace.workspaceForVertex(VertexIndex),
                       EdgeType::kDimension, VertexType::kDimension);

  typename VertexType::BVector perturbation_buffer(vertex->dimension());
  perturbation_buffer.fill(0.);
  g2o::VectorX::MapType perturbation(perturbation_buffer.data(),
                                     perturbation_buffer.size());

  for (int dimension = 0; dimension < vertex->dimension(); ++dimension) {
    vertex->push();
    perturbation[dimension] = step;
    vertex->oplus(perturbation);
    edge.computeError();
    auto error = edge.error();
    vertex->pop();

    vertex->push();
    perturbation[dimension] = -step;
    vertex->oplus(perturbation);
    edge.computeError();
    error -= edge.error();
    vertex->pop();

    perturbation[dimension] = 0.;
    jacobian.col(dimension) = error / (2. * step);
  }

  edge.computeError();
}

template <typename EdgeType>
void evaluateJacobianWithStep(EdgeType& edge,
                              g2o::JacobianWorkspace& analytic_workspace,
                              g2o::JacobianWorkspace& numeric_workspace,
                              double step,
                              const g2o::EpsilonFunction& epsilon) {
  edge.template BaseBinaryEdge<
      EdgeType::kDimension, typename EdgeType::Measurement,
      typename EdgeType::VertexXiType,
      typename EdgeType::VertexXjType>::linearizeOplus(numeric_workspace);
  analytic_workspace = numeric_workspace;

  fillNumericJacobian<0>(edge, numeric_workspace, step);
  fillNumericJacobian<1>(edge, numeric_workspace, step);

  for (int vertex_index = 0; vertex_index < 2; ++vertex_index) {
    const int num_elements =
        EdgeType::kDimension * (vertex_index == 0
                                    ? EdgeType::VertexXiType::kDimension
                                    : EdgeType::VertexXjType::kDimension);
    g2o::VectorX::ConstMapType numeric(
        numeric_workspace.workspaceForVertex(vertex_index), num_elements);
    g2o::VectorX::ConstMapType analytic(
        analytic_workspace.workspaceForVertex(vertex_index), num_elements);
    EXPECT_THAT(g2o::internal::print_wrap(analytic),
                g2o::internal::JacobianApproxEqual(
                    g2o::internal::print_wrap(numeric), epsilon));
  }
}
}  // namespace

template <>
struct g2o::internal::RandomValue<g2o::SBACam> {
  using Type = g2o::SBACam;
  static Type create() {
    g2o::SBACam result(g2o::Quaternion::UnitRandom(), g2o::Vector3::Random());
    return result;
  }
};

using SBAIoTypes = ::testing::Types<
    std::tuple<g2o::EdgeSE3Expmap>, std::tuple<g2o::EdgeSBAScale>,
    std::tuple<g2o::EdgeSBACam>, std::tuple<g2o::EdgeSE3ProjectXYZ>,
    std::tuple<g2o::EdgeSE3ProjectXYZOnlyPose>,
    std::tuple<g2o::EdgeStereoSE3ProjectXYZ>,
    std::tuple<g2o::EdgeStereoSE3ProjectXYZOnlyPose>,
    std::tuple<g2o::EdgeProjectP2SC>, std::tuple<g2o::EdgeProjectP2MC>>;
INSTANTIATE_TYPED_TEST_SUITE_P(SBA, FixedSizeEdgeBasicTests, SBAIoTypes,
                               g2o::internal::DefaultTypeNames);

TEST(SBA, EdgeSE3ExpmapJacobian) {
  auto v1 = std::make_shared<g2o::VertexSE3Expmap>();
  v1->setId(0);

  auto v2 = std::make_shared<g2o::VertexSE3Expmap>();
  v2->setId(1);

  g2o::EdgeSE3Expmap e;
  e.setVertex(0, v1);
  e.setVertex(1, v2);
  e.setInformation(g2o::EdgeSE3Expmap::InformationType::Identity());

  g2o::JacobianWorkspace jacobianWorkspace;
  g2o::JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 200; ++k) {
    v1->setEstimate(g2o::internal::RandomSE3Quat::create());
    v2->setEstimate(g2o::internal::RandomSE3Quat::create());
    e.setMeasurement(g2o::internal::RandomSE3Quat::create());

    evaluateJacobianWithStep(e, jacobianWorkspace, numericJacobianWorkspace,
                             1e-6,
                             [](const double, const double) { return 5e-3; });
  }
}
