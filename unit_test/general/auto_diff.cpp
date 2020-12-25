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

#include <cmath>

#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_fixed_sized_edge.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "gtest/gtest.h"

class VertexFlatSE2 : public g2o::BaseVertex<3, g2o::Vector3> {
 public:
  virtual void setToOriginImpl() { _estimate.setZero(); }

  virtual void oplusImpl(const number_t* update) {
    _estimate += Eigen::Map<const g2o::Vector3>(update);
    _estimate(2) = g2o::normalize_theta(_estimate(2));
  }

  virtual bool read(std::istream&) { return false; }
  virtual bool write(std::ostream&) const { return false; }
};

/**
 * A test case edge connecting three vertices.
 * Here we project a point along two poses and compare it against a measurement
 */
class Edge3ADTester : public g2o::BaseFixedSizedEdge<2, g2o::Vector2, VertexFlatSE2, VertexFlatSE2,
                                                     g2o::VertexPointXY> {
 public:
  using g2o::BaseFixedSizedEdge<2, g2o::Vector2, VertexFlatSE2, VertexFlatSE2,
                                g2o::VertexPointXY>::linearizeOplus;

  //! apply the SE2 pose on a point
  template <typename T>
  void project(const T* pose, const T* point, T* result) const {
    T cth = cos(pose[2]);
    T sth = sin(pose[2]);

    result[0] = pose[0] + cth * point[0] - sth * point[1];
    result[1] = pose[1] + sth * point[0] + cth * point[1];
  }

  //! implementation of the templetazed error function
  template <typename T>
  bool operator()(const T* p1, const T* p2, const T* point, T* error) const {
    T aux[2];
    project(p2, point, aux);
    T prediction[2];
    project(p1, aux, prediction);
    error[0] = prediction[0] - T(measurement()(0));
    error[1] = prediction[1] - T(measurement()(1));
    return true;
  }

  // add the AD interface
  G20_MAKE_AUTO_AD_FUNCTIONS

  // NOOPs
  virtual bool read(std::istream&) { return false; };
  virtual bool write(std::ostream&) const { return false; };
};

/**
 * Test ficture for performing the AD tests.
 * We setup three vertices and the edge to perform the tests
 */
class AutoDifferentiation : public ::testing::Test {
 protected:
  void SetUp() override {
    v1.setEstimate(g2o::Vector3(0, 0, 0));
    v2.setEstimate(g2o::Vector3(1, 1, 1));
    point.setEstimate(g2o::Vector2(2, 2));

    testEdge.setMeasurement(g2o::Vector2(0, 0));
    testEdge.setVertex(0, &v1);
    testEdge.setVertex(1, &v2);
    testEdge.setVertex(2, &point);

    jacobianWorkspace.updateSize(&testEdge);
    jacobianWorkspace.allocate();
  }
  VertexFlatSE2 v1;
  VertexFlatSE2 v2;
  g2o::VertexPointXY point;
  Edge3ADTester testEdge;
  g2o::JacobianWorkspace jacobianWorkspace;
};

TEST_F(AutoDifferentiation, ComputesSomething) {
  testEdge.linearizeOplus(jacobianWorkspace);

#if 0
  std::cerr << PVAR(testEdge.jacobianOplusXn<0>()) << std::endl;
  std::cerr << PVAR(testEdge.jacobianOplusXn<1>()) << std::endl;
  std::cerr << PVAR(testEdge.jacobianOplusXn<2>()) << std::endl;
#endif

  ASSERT_FALSE(testEdge.jacobianOplusXn<0>().array().isNaN().any()) << "Jacobian contains NaN";
  ASSERT_FALSE(testEdge.jacobianOplusXn<1>().array().isNaN().any()) << "Jacobian contains NaN";
  ASSERT_FALSE(testEdge.jacobianOplusXn<2>().array().isNaN().any()) << "Jacobian contains NaN";

  ASSERT_FALSE(testEdge.jacobianOplusXn<0>().array().isInf().any()) << "Jacobian not finite";
  ASSERT_FALSE(testEdge.jacobianOplusXn<1>().array().isInf().any()) << "Jacobian not finite";
  ASSERT_FALSE(testEdge.jacobianOplusXn<2>().array().isInf().any()) << "Jacobian not finite";

  ASSERT_LE(0, testEdge.jacobianOplusXn<0>().array().abs().maxCoeff()) << "Jacobian is zero";
  ASSERT_LE(0, testEdge.jacobianOplusXn<1>().array().abs().maxCoeff()) << "Jacobian is zero";
  ASSERT_LE(0, testEdge.jacobianOplusXn<2>().array().abs().maxCoeff()) << "Jacobian is zero";
}

TEST_F(AutoDifferentiation, ComputesNothingForFixed) {
  v2.setFixed(true);

  testEdge.linearizeOplus(jacobianWorkspace);

  ASSERT_LE(0, testEdge.jacobianOplusXn<0>().array().abs().maxCoeff()) << "Jacobian is zero";
  ASSERT_DOUBLE_EQ(0, testEdge.jacobianOplusXn<1>().array().abs().maxCoeff()) << "Jacobian is non-zero";
  ASSERT_LE(0, testEdge.jacobianOplusXn<2>().array().abs().maxCoeff()) << "Jacobian is zero";
}
