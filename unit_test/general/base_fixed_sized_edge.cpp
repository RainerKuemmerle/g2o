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

#include <gtest/gtest.h>

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_variable_sized_edge.h"
#include "g2o/core/eigen_types.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"

class Edge3Constant
    : public g2o::BaseFixedSizedEdge<2, g2o::Vector2, g2o::VertexSE2,
                                     g2o::VertexSE2, g2o::VertexPointXY> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Edge3Constant() = default;
  void computeError() override {
    const auto a = vertexXnRaw<0>()->estimate();
    const auto b = vertexXnRaw<1>()->estimate();
    const auto c = vertexXnRaw<2>()->estimate();
    error_ = (a * b * c - measurement_).eval();
  }
  bool read(std::istream&) override { return false; };
  bool write(std::ostream&) const override { return false; };
};

class Edge3Dynamic : public g2o::BaseVariableSizedEdge<2, g2o::Vector2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Edge3Dynamic() { resize(3); };
  void computeError() override {
    const auto a = static_cast<const g2o::VertexSE2*>(vertexRaw(0))->estimate();
    const auto b = static_cast<const g2o::VertexSE2*>(vertexRaw(1))->estimate();
    const auto c =
        static_cast<const g2o::VertexPointXY*>(vertexRaw(2))->estimate();
    error_ = (a * b * c - measurement_).eval();
  }
  bool read(std::istream&) override { return false; };
  bool write(std::ostream&) const override { return false; };
};

class VertexNotDefaultCtor : public g2o::BaseVertex<2, g2o::Vector2> {
 public:
  VertexNotDefaultCtor(int x, int y) { estimate_ = g2o::Vector2(x, y); }

  void oplusImpl(const g2o::VectorX::MapType& update) override {
    estimate_ += update.head<2>();
  }

  void setToOriginImpl() override { estimate_.setZero(); }
  bool read(std::istream& /*is*/) override { return false; };
  bool write(std::ostream& /*os*/) const override { return false; };
};

class EdgeUnaryCreateVertexTester
    : public g2o::BaseUnaryEdge<2, g2o::Vector2, VertexNotDefaultCtor> {
 public:
  EdgeUnaryCreateVertexTester() = default;

  void computeError() override {
    const VertexNotDefaultCtor* v = vertexXnRaw<0>();
    error_ = v->estimate() - measurement_;
  }
  bool read(std::istream& /*is*/) override { return false; };
  bool write(std::ostream& /*os*/) const override { return false; };

  void setMeasurement(const g2o::Vector2& m) override { measurement_ = m; }

  int measurementDimension() const override { return 2; }
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
  for (int j = 0; j < 32; ++j)
    for (int i = 0; i < j; ++i)
      ASSERT_EQ(index_to_pair(pair_to_index(i, j)), TrivialPair(i, j));
  for (int k = 0; k < 1024; ++k)
    ASSERT_EQ(pair_to_index(index_to_pair(k).first, index_to_pair(k).second),
              k);

  int k = 0;
  for (int j = 0; j < 32; ++j)
    for (int i = 0; i < j; ++i) {
      ASSERT_EQ(pair_to_index(i, j), k);
      ++k;
    }
}

TEST(General, ConstantEdgeConstructor) {
  ASSERT_EQ(typeid(Edge3Dynamic::ErrorVector),
            typeid(Edge3Constant::ErrorVector));
  ASSERT_EQ(typeid(Edge3Dynamic::InformationType),
            typeid(Edge3Constant::InformationType));

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

  auto* v1 = e.createVertex(0);
  auto* v2 = e.createVertex(1);
  auto* v3 = e.createVertex(2);
  ASSERT_EQ(typeid(*v1), typeid(g2o::VertexSE2));
  ASSERT_EQ(typeid(*v2), typeid(g2o::VertexSE2));
  ASSERT_EQ(typeid(*v3), typeid(g2o::VertexPointXY));
  delete v3;
  delete v2;
  delete v1;

  ASSERT_EQ(nullptr, e.createVertex(-1));
  ASSERT_EQ(nullptr, e.createVertex(3));
}

TEST(General, FixedEdgeCreateVertexNonDefaultCtor) {
  EdgeUnaryCreateVertexTester edge;
  auto* vertex = edge.createVertex(0, 42, 23);
  EXPECT_NE(vertex, nullptr);
  ASSERT_EQ(typeid(*vertex), typeid(VertexNotDefaultCtor));
  auto* casted_vertex = dynamic_cast<VertexNotDefaultCtor*>(vertex);
  ASSERT_NE(casted_vertex, nullptr);
  EXPECT_TRUE(casted_vertex->estimate().isApprox(g2o::Vector2(42, 23)));
}

template <typename EdgeType>
class EdgeTester {
 public:
  EdgeTester() {
    edge.setMeasurement(g2o::Vector2{.3, .4});
    edge.setInformation(g2o::Matrix2::Identity());

    v1->setId(0);
    v1->setEstimate(g2o::SE2(.1, .2, .3));
    v2->setId(1);
    v2->setEstimate(g2o::SE2(.3, .1, .2));
    v3->setId(2);
    v3->setEstimate(g2o::Vector2(-.3, .5));
    edge.setVertex(0, v1);
    edge.setVertex(1, v2);
    edge.setVertex(2, v3);

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
    v1->mapHessianMemory(hessian00.data());
    v2->mapHessianMemory(hessian11.data());
    v3->mapHessianMemory(hessian22.data());
  }

  EdgeType edge;

  std::shared_ptr<g2o::VertexSE2> v1 = std::make_shared<g2o::VertexSE2>();
  std::shared_ptr<g2o::VertexSE2> v2 = std::make_shared<g2o::VertexSE2>();
  std::shared_ptr<g2o::VertexPointXY> v3 =
      std::make_shared<g2o::VertexPointXY>();

  g2o::JacobianWorkspace jacobianWorkspace;

  Eigen::Matrix<number_t, 3, 3> hessian01;
  Eigen::Matrix<number_t, 3, 2> hessian02;
  Eigen::Matrix<number_t, 3, 2> hessian12;
  Eigen::Matrix<number_t, 3, 3> hessian00;
  Eigen::Matrix<number_t, 3, 3> hessian11;
  Eigen::Matrix<number_t, 2, 2> hessian22;
};

TEST(ConstantEdgeTest, ConstantEdgeAllVerticesFixed) {
  const EdgeTester<Edge3Dynamic> dynamic;
  const EdgeTester<Edge3Constant> constant;
  ASSERT_EQ(dynamic.edge.allVerticesFixed(), constant.edge.allVerticesFixed());
  ASSERT_FALSE(constant.edge.allVerticesFixed());
  dynamic.v1->setFixed(true);
  dynamic.v2->setFixed(true);
  dynamic.v3->setFixed(true);
  constant.v1->setFixed(true);
  constant.v2->setFixed(true);
  constant.v3->setFixed(true);
  ASSERT_EQ(dynamic.edge.allVerticesFixed(), constant.edge.allVerticesFixed());
  ASSERT_TRUE(constant.edge.allVerticesFixed());
}

TEST(ConstantEdgeTest, ConstantEdgeComputeError) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.computeError();
  constant.edge.computeError();
  EXPECT_DOUBLE_EQ(0.0, (dynamic.edge.error() - constant.edge.error()).norm());
}

TEST(ConstantEdgeTest, ConstantEdgeLinearizeOplus) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;
  dynamic.edge.computeError();
  constant.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  constant.edge.linearizeOplus(constant.jacobianWorkspace);
  EXPECT_DOUBLE_EQ(0.0,
                   (Eigen::Map<g2o::MatrixX>(
                        dynamic.jacobianWorkspace.workspaceForVertex(0), 2, 3) -
                    Eigen::Map<g2o::MatrixX>(
                        constant.jacobianWorkspace.workspaceForVertex(0), 2, 3))
                       .norm());
  EXPECT_DOUBLE_EQ(0.0,
                   (Eigen::Map<g2o::MatrixX>(
                        dynamic.jacobianWorkspace.workspaceForVertex(1), 2, 3) -
                    Eigen::Map<g2o::MatrixX>(
                        constant.jacobianWorkspace.workspaceForVertex(1), 2, 3))
                       .norm());
  EXPECT_DOUBLE_EQ(0.0,
                   (Eigen::Map<g2o::MatrixX>(
                        dynamic.jacobianWorkspace.workspaceForVertex(2), 2, 2) -
                    Eigen::Map<g2o::MatrixX>(
                        constant.jacobianWorkspace.workspaceForVertex(2), 2, 2))
                       .norm());
}

TEST(ConstantEdgeTest, ConstantEdgeConstructQuadraticForm) {
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

TEST(ConstantEdgeTest, ConstantEdgeConstructQuadraticFormRobust) {
  EdgeTester<Edge3Dynamic> dynamic;
  EdgeTester<Edge3Constant> constant;

  dynamic.edge.setMeasurement(g2o::Vector2{.3, 3.4});
  constant.edge.setMeasurement(g2o::Vector2{.3, 3.4});

  auto rk_dynamic = std::make_shared<g2o::RobustKernelHuber>();
  dynamic.edge.setRobustKernel(rk_dynamic);
  dynamic.edge.computeError();
  dynamic.edge.linearizeOplus(dynamic.jacobianWorkspace);
  dynamic.edge.constructQuadraticForm();

  auto rk_constant = std::make_shared<g2o::RobustKernelHuber>();
  constant.edge.setRobustKernel(rk_constant);
  constant.edge.computeError();
  constant.edge.linearizeOplus(constant.jacobianWorkspace);
  constant.edge.constructQuadraticForm();
  EXPECT_NEAR(0, (dynamic.edge.error() - constant.edge.error()).norm(), 1e-7);
  EXPECT_DOUBLE_EQ(0.0,
                   (Eigen::Map<g2o::MatrixX>(
                        dynamic.jacobianWorkspace.workspaceForVertex(0), 2, 3) -
                    Eigen::Map<g2o::MatrixX>(
                        constant.jacobianWorkspace.workspaceForVertex(0), 2, 3))
                       .norm());
  EXPECT_NEAR(0.0, (dynamic.hessian00 - constant.hessian00).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian11 - constant.hessian11).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian22 - constant.hessian22).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian01 - constant.hessian01).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian02 - constant.hessian02).norm(), 1e-7);
  EXPECT_NEAR(0.0, (dynamic.hessian12 - constant.hessian12).norm(), 1e-7);
}

TEST(ConstantEdgeTest, ConstantEdgeConstructQuadraticFormRowMajor) {
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
  EXPECT_NEAR(
      0.0,
      (constant_colMajor.hessian01 - constant.hessian01.transpose()).norm(),
      1e-7);
  EXPECT_NEAR(
      0.0,
      (constant_colMajor.hessian02 - hessian20_constant.transpose()).norm(),
      1e-7);
  EXPECT_NEAR(
      0.0,
      (constant_colMajor.hessian12 - hessian21_constant.transpose()).norm(),
      1e-7);
}
