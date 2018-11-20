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

#include "g2o/core/base_constant_edge.h"
#include "g2o/core/base_multi_edge.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"

// create 2 classes with 3 vertices, one based on multi and one based on constant edge class
// try out all members

class Edge3Constant : public g2o::BaseConstantEdge<2, g2o::Vector2, g2o::VertexSE2, g2o::VertexSE2, g2o::VertexPointXY>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Edge3Constant() : g2o::BaseConstantEdge<2, g2o::Vector2, g2o::VertexSE2, g2o::VertexSE2, g2o::VertexPointXY>() {};
    void computeError()
    {
      const auto a = static_cast<const g2o::VertexSE2*>(_vertices[0])->estimate();
      const auto b = static_cast<const g2o::VertexSE2*>(_vertices[1])->estimate();
      const auto c = static_cast<const g2o::VertexPointXY*>(_vertices[2])->estimate();
      _error = (a * b * c - _measurement).eval();
    }
    virtual bool read(std::istream&) {return false;};
    virtual bool write(std::ostream&) const {return false;};
};

class Edge3Dynamic : public g2o::BaseMultiEdge<2, g2o::Vector2>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Edge3Dynamic() : g2o::BaseMultiEdge<2, g2o::Vector2>()
    {
      resize(3);
    };
    void computeError()
    {
      const auto a = static_cast<const g2o::VertexSE2*>(_vertices[0])->estimate();
      const auto b = static_cast<const g2o::VertexSE2*>(_vertices[1])->estimate();
      const auto c = static_cast<const g2o::VertexPointXY*>(_vertices[2])->estimate();
      _error = (a * b * c - _measurement).eval();
    }
    virtual bool read(std::istream&) {return false;};
    virtual bool write(std::ostream&) const {return false;};
};

TEST(General, IndexToPairToIndex)
{
  using g2o::internal::pair_to_index;
  using g2o::internal::index_to_pair;
  ASSERT_EQ(index_to_pair(0).first, 0);
  ASSERT_EQ(index_to_pair(0).second, 1);
  ASSERT_EQ(index_to_pair(1).first, 0);
  ASSERT_EQ(index_to_pair(1).second, 2);
  ASSERT_EQ(index_to_pair(2).first, 1);
  ASSERT_EQ(index_to_pair(2).second, 2);
  ASSERT_EQ(pair_to_index(0, 1), 0);
  ASSERT_EQ(pair_to_index(0, 2), 1);
  ASSERT_EQ(pair_to_index(1, 2), 2);
  ASSERT_EQ(pair_to_index(index_to_pair(0).first, index_to_pair(0).second), 0);
  ASSERT_EQ(pair_to_index(index_to_pair(1).first, index_to_pair(1).second), 1);
  ASSERT_EQ(pair_to_index(index_to_pair(2).first, index_to_pair(2).second), 2);
  ASSERT_EQ(pair_to_index(index_to_pair(10).first, index_to_pair(10).second), 10);
  ASSERT_EQ(index_to_pair(pair_to_index(0, 1)), std::make_pair(0, 1));
  ASSERT_EQ(index_to_pair(pair_to_index(0, 2)), std::make_pair(0, 2));
  ASSERT_EQ(index_to_pair(pair_to_index(1, 2)), std::make_pair(1, 2));
}

TEST(General, ConstantEdgeConstructor)
{
  ASSERT_EQ(typeid(Edge3Dynamic::ErrorVector), typeid(Edge3Constant::ErrorVector));
  ASSERT_EQ(typeid(Edge3Dynamic::InformationType), typeid(Edge3Constant::InformationType));

  Edge3Constant e_constant;
  Edge3Dynamic e_dynamic;
  ASSERT_EQ(e_dynamic.vertices()[0], e_constant.vertices()[0]);
  ASSERT_EQ(e_dynamic.vertices()[1], e_constant.vertices()[1]);
  ASSERT_EQ(e_dynamic.vertices()[2], e_constant.vertices()[2]);
}

TEST(General, ConstantEdgeJacobians)
{
  Edge3Constant e_constant;
  e_constant.setMeasurement(g2o::Vector2{.3, .4});
  e_constant.setInformation(g2o::Matrix2::Identity());
  Edge3Dynamic e_dynamic;
  e_dynamic.setMeasurement(g2o::Vector2{.3, .4});
  e_dynamic.setInformation(g2o::Matrix2::Identity());
  auto* v1 = new g2o::VertexSE2();
  v1->setId(0);
  v1->setEstimate(g2o::SE2(.1, .2, .3));
  auto* v2 = new g2o::VertexSE2();
  v2->setId(1);
  v2->setEstimate(g2o::SE2(.3, .1, .2));
  auto* v3 = new g2o::VertexPointXY();
  v3->setId(2);
  v3->setEstimate(g2o::Vector2(-.3, .5));
  e_dynamic.setVertex(0, v1);
  e_dynamic.setVertex(1, v2);
  e_dynamic.setVertex(2, v3);
  e_constant.setVertex(0, v1);
  e_constant.setVertex(1, v2);
  e_constant.setVertex(2, v3);

  e_dynamic.computeError();
  e_constant.computeError();
  ASSERT_DOUBLE_EQ(0.0, (e_dynamic.error() - e_constant.error()).norm());

  g2o::JacobianWorkspace jacobianWorkspace_dynamic;
  jacobianWorkspace_dynamic.updateSize(&e_dynamic);
  jacobianWorkspace_dynamic.allocate();
  e_dynamic.linearizeOplus(jacobianWorkspace_dynamic);

  g2o::JacobianWorkspace jacobianWorkspace_constant;
  jacobianWorkspace_constant.updateSize(&e_constant);
  jacobianWorkspace_constant.allocate();
  e_constant.linearizeOplus(jacobianWorkspace_constant);

  ASSERT_DOUBLE_EQ(0.0, (Eigen::Map<g2o::MatrixX>(jacobianWorkspace_dynamic.workspaceForVertex(0), 2, 3)
                       - Eigen::Map<g2o::MatrixX>(jacobianWorkspace_constant.workspaceForVertex(0), 2, 3)).norm());
  ASSERT_DOUBLE_EQ(0.0, (Eigen::Map<g2o::MatrixX>(jacobianWorkspace_dynamic.workspaceForVertex(1), 2, 3)
                       - Eigen::Map<g2o::MatrixX>(jacobianWorkspace_constant.workspaceForVertex(1), 2, 3)).norm());
  ASSERT_DOUBLE_EQ(0.0, (Eigen::Map<g2o::MatrixX>(jacobianWorkspace_dynamic.workspaceForVertex(2), 2, 2)
                       - Eigen::Map<g2o::MatrixX>(jacobianWorkspace_constant.workspaceForVertex(2), 2, 2)).norm());


  ASSERT_EQ(e_dynamic.allVerticesFixed(), e_constant.allVerticesFixed());
  ASSERT_FALSE(e_constant.allVerticesFixed());
  v1->setFixed(true);
  v2->setFixed(true);
  v3->setFixed(true);
  ASSERT_EQ(e_dynamic.allVerticesFixed(), e_constant.allVerticesFixed());
  ASSERT_TRUE(e_constant.allVerticesFixed());
  v3->setFixed(false);
  v2->setFixed(false);
  v1->setFixed(false);

  Eigen::Matrix<number_t, 3, 3> hessian01_dynamic = Eigen::Matrix<number_t, 3, 3>::Zero();
  Eigen::Matrix<number_t, 3, 2> hessian02_dynamic = Eigen::Matrix<number_t, 3, 2>::Zero();
  Eigen::Matrix<number_t, 3, 2> hessian12_dynamic = Eigen::Matrix<number_t, 3, 2>::Zero();
  e_dynamic.mapHessianMemory(hessian01_dynamic.data(), 0, 1, false);
  e_dynamic.mapHessianMemory(hessian02_dynamic.data(), 0, 2, false);
  e_dynamic.mapHessianMemory(hessian12_dynamic.data(), 1, 2, false);

  Eigen::Matrix<number_t, 3, 3> hessian00_dynamic = Eigen::Matrix<number_t, 3, 3>::Zero();
  Eigen::Matrix<number_t, 3, 3> hessian11_dynamic = Eigen::Matrix<number_t, 3, 3>::Zero();
  Eigen::Matrix<number_t, 2, 2> hessian22_dynamic = Eigen::Matrix<number_t, 2, 2>::Zero();
  v1->mapHessianMemory(hessian00_dynamic.data());
  v2->mapHessianMemory(hessian11_dynamic.data());
  v3->mapHessianMemory(hessian22_dynamic.data());

  e_dynamic.constructQuadraticForm();

  Eigen::Matrix<number_t, 3, 3> hessian01_constant = Eigen::Matrix<number_t, 3, 3>::Zero();
  Eigen::Matrix<number_t, 3, 2> hessian02_constant = Eigen::Matrix<number_t, 3, 2>::Zero();
  Eigen::Matrix<number_t, 3, 2> hessian12_constant = Eigen::Matrix<number_t, 3, 2>::Zero();
  e_constant.mapHessianMemory(hessian01_constant.data(), 0, 1, false);
  e_constant.mapHessianMemory(hessian02_constant.data(), 0, 2, false);
  e_constant.mapHessianMemory(hessian12_constant.data(), 1, 2, false);

  Eigen::Matrix<number_t, 3, 3> hessian00_constant = Eigen::Matrix<number_t, 3, 3>::Zero();
  Eigen::Matrix<number_t, 3, 3> hessian11_constant = Eigen::Matrix<number_t, 3, 3>::Zero();
  Eigen::Matrix<number_t, 2, 2> hessian22_constant = Eigen::Matrix<number_t, 2, 2>::Zero();
  v1->mapHessianMemory(hessian00_constant.data());
  v2->mapHessianMemory(hessian11_constant.data());
  v3->mapHessianMemory(hessian22_constant.data());

  e_constant.constructQuadraticForm();

  ASSERT_DOUBLE_EQ(0.0, (hessian00_dynamic - hessian00_constant).norm());
  ASSERT_DOUBLE_EQ(0.0, (hessian11_dynamic - hessian11_constant).norm());
  ASSERT_DOUBLE_EQ(0.0, (hessian22_dynamic - hessian22_constant).norm());
  ASSERT_DOUBLE_EQ(0.0, (hessian01_dynamic - hessian01_constant).norm());


  // check rowMajor
}

