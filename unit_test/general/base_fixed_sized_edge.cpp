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

#include "g2o/core/base_fixed_sized_edge.h"

#include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gtest/gtest.h"

class Edge3Constant : public g2o::BaseFixedSizedEdge<2, g2o::Vector2, g2o::VertexSE2,
                                                     g2o::VertexSE2, g2o::VertexPointXY> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Edge3Constant()
      : g2o::BaseFixedSizedEdge<2, g2o::Vector2, g2o::VertexSE2, g2o::VertexSE2,
                                g2o::VertexPointXY>(){};
  void computeError() {
    const auto a = static_cast<const g2o::VertexSE2*>(_vertices[0])->estimate();
    const auto b = static_cast<const g2o::VertexSE2*>(_vertices[1])->estimate();
    const auto c = static_cast<const g2o::VertexPointXY*>(_vertices[2])->estimate();
    _error = (a * b * c - _measurement).eval();
  }
  virtual bool read(std::istream&) { return false; };
  virtual bool write(std::ostream&) const { return false; };
};

class Edge3Dynamic : public g2o::BaseVariableSizedEdge<2, g2o::Vector2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Edge3Dynamic() : g2o::BaseVariableSizedEdge<2, g2o::Vector2>() { resize(3); };
  void computeError() {
    const auto a = static_cast<const g2o::VertexSE2*>(_vertices[0])->estimate();
    const auto b = static_cast<const g2o::VertexSE2*>(_vertices[1])->estimate();
    const auto c = static_cast<const g2o::VertexPointXY*>(_vertices[2])->estimate();
    _error = (a * b * c - _measurement).eval();
  }
  virtual bool read(std::istream&) { return false; };
  virtual bool write(std::ostream&) const { return false; };
};

TEST(General, IndexToPairToIndex) {
  using g2o::internal::index_to_pair;
  using g2o::internal::pair_to_index;
  using g2o::internal::TrivialPair;

  ASSERT_EQ(index_to_pair(0).first, 0);
  ASSERT_EQ(index_to_pair(0).second, 1);
  ASSERT_EQ(index_to_pair(1).first, 0);
  ASSERT_EQ(index_to_pair(1).second, 2);
  ASSERT_EQ(index_to_pair(2).first, 1);
  ASSERT_EQ(index_to_pair(2).second, 2);
  ASSERT_EQ(pair_to_index(0, 1), 0);
  ASSERT_EQ(pair_to_index(0, 2), 1);
  ASSERT_EQ(pair_to_index(1, 2), 2);
  for(int j = 0; j < 32; ++j)
    for(int i = 0; i < j; ++i)
      ASSERT_EQ(index_to_pair(pair_to_index(i, j)), TrivialPair(i, j));
  for(int k = 0; k < 1024; ++k)
    ASSERT_EQ(pair_to_index(index_to_pair(k).first, index_to_pair(k).second), k);

  int k = 0;
  for (int j = 0; j < 32; ++j)
    for (int i = 0; i < j; ++i) {
      ASSERT_EQ(pair_to_index(i, j), k);
      ++k;
    }
}

TEST(General, ConstantEdgeConstructor) {
  ASSERT_EQ(typeid(Edge3Dynamic::ErrorVector), typeid(Edge3Constant::ErrorVector));
  ASSERT_EQ(typeid(Edge3Dynamic::InformationType), typeid(Edge3Constant::InformationType));

  Edge3Constant e_constant;
  ASSERT_EQ(e_constant.vertices()[0], nullptr);
  ASSERT_EQ(e_constant.vertices()[1], nullptr);
  ASSERT_EQ(e_constant.vertices()[2], nullptr);
  Edge3Dynamic e_dynamic;
  ASSERT_EQ(e_dynamic.vertices()[0], e_constant.vertices()[0]);
  ASSERT_EQ(e_dynamic.vertices()[1], e_constant.vertices()[1]);
  ASSERT_EQ(e_dynamic.vertices()[2], e_constant.vertices()[2]);
}

TEST(General, FixedEdgeCreateVertex) {
  Edge3Constant e;

  auto v1 = e.createVertex(0);
  auto v2 = e.createVertex(1);
  auto v3 = e.createVertex(2);
  ASSERT_EQ(typeid(*v1), typeid(g2o::VertexSE2));
  ASSERT_EQ(typeid(*v2), typeid(g2o::VertexSE2));
  ASSERT_EQ(typeid(*v3), typeid(g2o::VertexPointXY));
  delete v3;
  delete v2;
  delete v1;

  ASSERT_EQ(nullptr, e.createVertex(-1));
  ASSERT_EQ(nullptr, e.createVertex(3));
}

template <typename EdgeType>
class EdgeTester {
 public:
  EdgeTester() {
    edge.setMeasurement(g2o::Vector2{.3, .4});
    edge.setInformation(g2o::Matrix2::Identity());

    v1.setId(0);
    v1.setEstimate(g2o::SE2(.1, .2, .3));
    v2.setId(1);
    v2.setEstimate(g2o::SE2(.3, .1, .2));
    v3.setId(2);
    v3.setEstimate(g2o::Vector2(-.3, .5));
    edge.setVertex(0, &v1);
    edge.setVertex(1, &v2);
    edge.setVertex(2, &v3);

    jacobianWorkspace.updateSize(&edge);
    jacobianWorkspace.allocate();

    hessian01.setZero();
    hessian02.setZero();
    hessian12.setZero();
    hessian00.setZero();
    hessian11.setZero();
    hessian22.setZero();
    edge.mapHessianMemory(hessian01.data(), 0, 1, false);
    edge.mapHessianMemory(hessian02.data(), 0, 2, false);
    edge.mapHessianMemory(hessian12.data(), 1, 2, false);
    v1.mapHessianMemory(hessian00.data());
    v2.mapHessianMemory(hessian11.data());
    v3.mapHessianMemory(hessian22.data());
  }

  EdgeType edge;

  g2o::VertexSE2 v1;
  g2o::VertexSE2 v2;
  g2o::VertexPointXY v3;

  g2o::JacobianWorkspace jacobianWorkspace;

  Eigen::Matrix<number_t, 3, 3> hessian01;
  Eigen::Matrix<number_t, 3, 2> hessian02;
  Eigen::Matrix<number_t, 3, 2> hessian12;
  Eigen::Matrix<number_t, 3, 3> hessian00;
  Eigen::Matrix<number_t, 3, 3> hessian11;
  Eigen::Matrix<number_t, 2, 2> hessian22;
};

TEST(ConstantEdgeTest, ConstantEdge_allVerticesFixed) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  ASSERT_EQ(dynamic.edge.allVerticesFixed(), constant.edge.allVerticesFixed());
  ASSERT_FALSE(constant.edge.allVerticesFixed());
  dynamic.v1.setFixed(true);
  dynamic.v2.setFixed(true);
  dynamic.v3.setFixed(true);
  constant.v1.setFixed(true);
  constant.v2.setFixed(true);
  constant.v3.setFixed(true);
  ASSERT_EQ(dynamic.edge.allVerticesFixed(), constant.edge.allVerticesFixed());
  ASSERT_TRUE(constant.edge.allVerticesFixed());
}

TEST(ConstantEdgeTest, ConstantEdge_computeError) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.computeError();
  constant.edge.computeError();
  EXPECT_DOUBLE_EQ(0.0, (dynamic.edge.error() - constant.edge.error()).norm());
}

TEST(ConstantEdgeTest, ConstantEdge_linearizeOplus) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.computeError();
  constant.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  constant.edge.linearizeOplus(constant.jacobianWorkspace);
  EXPECT_DOUBLE_EQ(
      0.0, (Eigen::Map<g2o::MatrixX>(dynamic.jacobianWorkspace.workspaceForVertex(0), 2, 3) -
            Eigen::Map<g2o::MatrixX>(constant.jacobianWorkspace.workspaceForVertex(0), 2, 3))
               .norm());
  EXPECT_DOUBLE_EQ(
      0.0, (Eigen::Map<g2o::MatrixX>(dynamic.jacobianWorkspace.workspaceForVertex(1), 2, 3) -
            Eigen::Map<g2o::MatrixX>(constant.jacobianWorkspace.workspaceForVertex(1), 2, 3))
               .norm());
  EXPECT_DOUBLE_EQ(
      0.0, (Eigen::Map<g2o::MatrixX>(dynamic.jacobianWorkspace.workspaceForVertex(2), 2, 2) -
            Eigen::Map<g2o::MatrixX>(constant.jacobianWorkspace.workspaceForVertex(2), 2, 2))
               .norm());
}

TEST(ConstantEdgeTest, ConstantEdge_constructQuadraticForm) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.computeError();
  constant.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  constant.edge.linearizeOplus(constant.jacobianWorkspace);

  dynamic.edge.constructQuadraticForm();
  constant.edge.constructQuadraticForm();

  EXPECT_NEAR(0.0, (dynamic.hessian00 - constant.hessian00).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian11 - constant.hessian11).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian22 - constant.hessian22).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian01 - constant.hessian01).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian02 - constant.hessian02).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian12 - constant.hessian12).norm(), 1e-7);
}

TEST(ConstantEdgeTest, ConstantEdge_constructQuadraticForm_robust) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;

  dynamic.edge.setMeasurement(g2o::Vector2{.3, 3.4});
  constant.edge.setMeasurement(g2o::Vector2{.3, 3.4});

  // assuming that G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS is false
  g2o::RobustKernelHuber* rk_dynamic = new g2o::RobustKernelHuber;
  dynamic.edge.setRobustKernel(rk_dynamic);
  dynamic.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  dynamic.edge.constructQuadraticForm();

  g2o::RobustKernelHuber* rk_constant = new g2o::RobustKernelHuber;
  constant.edge.setRobustKernel(rk_constant);
  constant.edge.computeError();
  constant.edge.linearizeOplus(constant.jacobianWorkspace);
  constant.edge.constructQuadraticForm();
  EXPECT_NEAR(0, (dynamic.edge.error() - constant.edge.error()).norm(), 1e-7);
  EXPECT_DOUBLE_EQ(
      0.0, (Eigen::Map<g2o::MatrixX>(dynamic.jacobianWorkspace.workspaceForVertex(0), 2, 3) -
            Eigen::Map<g2o::MatrixX>(constant.jacobianWorkspace.workspaceForVertex(0), 2, 3))
               .norm());
  EXPECT_NEAR(0.0, (dynamic.hessian00 - constant.hessian00).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian11 - constant.hessian11).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian22 - constant.hessian22).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian01 - constant.hessian01).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian02 - constant.hessian02).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian12 - constant.hessian12).norm(), 1e-7);
}

TEST(ConstantEdgeTest, ConstantEdge_constructQuadraticForm_rowMajor) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.mapHessianMemory(dynamic.hessian01.data(), 0, 1, true);
  Eigen::Matrix<number_t, 2, 3> hessian20_dynamic;
  Eigen::Matrix<number_t, 2, 3> hessian21_dynamic;
  hessian20_dynamic.setZero();
  hessian21_dynamic.setZero();
  dynamic.edge.mapHessianMemory(hessian20_dynamic.data(), 0, 2, true);
  dynamic.edge.mapHessianMemory(hessian21_dynamic.data(), 1, 2, true);

  constant.edge.mapHessianMemory(constant.hessian01.data(), 0, 1, true);
  Eigen::Matrix<number_t, 2, 3> hessian20_constant;
  Eigen::Matrix<number_t, 2, 3> hessian21_constant;
  hessian20_constant.setZero();
  hessian21_constant.setZero();
  constant.edge.mapHessianMemory(hessian20_constant.data(), 0, 2, true);
  constant.edge.mapHessianMemory(hessian21_constant.data(), 1, 2, true);

  dynamic.edge.computeError();
  constant.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  constant.edge.linearizeOplus(constant.jacobianWorkspace);

  dynamic.edge.constructQuadraticForm();
  constant.edge.constructQuadraticForm();

  EXPECT_NEAR(0.0, (dynamic.hessian00 - constant.hessian00).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian11 - constant.hessian11).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian22 - constant.hessian22).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian01 - constant.hessian01).norm(), 1e-7);
  EXPECT_NEAR(0.0, (hessian20_dynamic - hessian20_constant).norm(), 1e-7);
  EXPECT_NEAR(0.0, (hessian21_dynamic - hessian21_constant).norm(), 1e-7);

  EdgeTester<Edge3Constant> constant_colMajor;
  constant_colMajor.edge.computeError();
  constant_colMajor.edge.linearizeOplus(constant_colMajor.jacobianWorkspace);
  constant_colMajor.edge.constructQuadraticForm();
  EXPECT_NEAR(0.0, (constant_colMajor.hessian01 - constant.hessian01.transpose()).norm(), 1e-7);
  EXPECT_NEAR(0.0, (constant_colMajor.hessian02 - hessian20_constant.transpose()).norm(), 1e-7);
  EXPECT_NEAR(0.0, (constant_colMajor.hessian12 - hessian21_constant.transpose()).norm(), 1e-7);
}
