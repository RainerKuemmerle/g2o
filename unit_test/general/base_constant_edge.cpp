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

TEST(General, ConstantEdgeConstructor)
{
  Edge3Constant e_constant;
  Edge3Dynamic e_dynamic;
  ASSERT_EQ(e_dynamic.vertices()[0], e_constant.vertices()[0]);
  ASSERT_EQ(e_dynamic.vertices()[1], e_constant.vertices()[1]);
  ASSERT_EQ(e_dynamic.vertices()[2], e_constant.vertices()[2]);
  // todo, check type:
  //ASSERT_EQ(e_dynamic.ErrorVector, e_constant.ErrorVector);
  //ASSERT_EQ(e_dynamic.InformationType, e_constant.InformationType);
  //ASSERT_EQ(e_dynamic.allVerticesFixed(), e_constant.allVerticesFixed());
}

TEST(General, ConstantEdgeJacobians)
{
  Edge3Constant e_constant;
  Edge3Dynamic e_dynamic;
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
  e_dynamic.setVertex(2, v2);
  e_constant.setVertex(0, v1);
  e_constant.setVertex(1, v2);
  e_constant.setVertex(2, v2);
  ASSERT_EQ(e_dynamic.allVerticesFixed(), e_constant.allVerticesFixed());
}

