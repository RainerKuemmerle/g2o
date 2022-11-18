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
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "gtest/gtest.h"

class VertexFlatSE2 : public g2o::BaseVertex<3, g2o::Vector3> {
 public:
  void setToOriginImpl() override { estimate_.setZero(); }

  void oplusImpl(const g2o::VectorX::MapType& update) override {
    estimate_ += update.head<kDimension>();
    estimate_(2) = g2o::normalize_theta(estimate_(2));
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

/**
 * A test case edge connecting three vertices.
 * Here we project a point along two poses and compare it against a measurement
 */
class Edge3ADTester
    : public g2o::BaseFixedSizedEdge<2, g2o::Vector2, VertexFlatSE2,
                                     VertexFlatSE2, g2o::VertexPointXY> {
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

  //! implementation of the templatized error function
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
  G2O_MAKE_AUTO_AD_FUNCTIONS

  // NOOPs
  bool read(std::istream&) override { return false; };
  bool write(std::ostream&) const override { return false; };
};

/**
 * Test implementation of an EdgeSE2 that implements operator() for AD
 */
class EdgeSE2AD : public g2o::EdgeSE2 {
 public:
  //! re-implementation of the templatized error function
  template <typename T>
  bool operator()(const T* p1, const T* p2, T* error) const {
    //// original implementation
    // SE2 delta = inverseMeasurement_ *
    // (v1->estimate().inverse()*v2->estimate()); _error = delta.toVector();

    Iso2<T> p1Iso = pointerToIso(p1);
    Iso2<T> p2Iso = pointerToIso(p2);
    g2o::VectorN<3, T> invMeasAsVector =
        inverseMeasurement_.toVector().cast<T>();
    Iso2<T> invMeas = pointerToIso(invMeasAsVector.data());

    Iso2<T> delta = invMeas * (p1Iso.inverse() * p2Iso);
    isoToPointer(delta, error);
    return true;
  }

  using g2o::BaseBinaryEdge<3, g2o::SE2, g2o::VertexSE2,
                            g2o::VertexSE2>::linearizeOplus;
  G2O_MAKE_AUTO_AD_LINEARIZEOPLUS_BY_GET

 protected:
  template <typename T>
  using Iso2 = Eigen::Transform<T, 2, Eigen::Isometry, Eigen::ColMajor>;

  template <typename T>
  static Iso2<T> pointerToIso(const T* p) {
    T cth = cos(p[2]);
    T sth = sin(p[2]);
    Iso2<T> iso = Iso2<T>::Identity();
    iso.linear() << cth, -sth, sth, cth;
    iso.translation()(0) = p[0];
    iso.translation()(1) = p[1];
    return iso;
  }

  template <typename T>
  static void isoToPointer(Iso2<T> iso, T* p) {
    p[0] = iso.translation()(0);
    p[1] = iso.translation()(1);
    p[2] = atan2(iso.linear()(1, 0), iso.linear()(0, 0));
  }
};

/**
 * Test fixture for performing the AD tests.
 * We setup three vertices and the edge to perform the tests
 */
class AutoDifferentiation : public ::testing::Test {
 protected:
  void SetUp() override {
    v1_->setEstimate(g2o::Vector3(0, 0, 0));
    v2_->setEstimate(g2o::Vector3(1, 1, 1));
    point_->setEstimate(g2o::Vector2(2, 2));

    testEdge_.setMeasurement(g2o::Vector2(0, 0));
    testEdge_.setVertex(0, v1_);
    testEdge_.setVertex(1, v2_);
    testEdge_.setVertex(2, point_);

    jacobianWorkspace_.updateSize(&testEdge_);
    jacobianWorkspace_.allocate();
  }
  std::shared_ptr<VertexFlatSE2> v1_ = std::make_shared<VertexFlatSE2>();
  std::shared_ptr<VertexFlatSE2> v2_ = std::make_shared<VertexFlatSE2>();
  std::shared_ptr<g2o::VertexPointXY> point_ =
      std::make_shared<g2o::VertexPointXY>();
  Edge3ADTester testEdge_;
  g2o::JacobianWorkspace jacobianWorkspace_;
};

TEST_F(AutoDifferentiation, ComputesSomething) {
  testEdge_.linearizeOplus(jacobianWorkspace_);

#if 0
  std::cerr << PVAR(testEdge.jacobianOplusXn<0>()) << std::endl;
  std::cerr << PVAR(testEdge.jacobianOplusXn<1>()) << std::endl;
  std::cerr << PVAR(testEdge.jacobianOplusXn<2>()) << std::endl;
#endif

  ASSERT_FALSE(testEdge_.jacobianOplusXn<0>().array().isNaN().any())
      << "Jacobian contains NaN";
  ASSERT_FALSE(testEdge_.jacobianOplusXn<1>().array().isNaN().any())
      << "Jacobian contains NaN";
  ASSERT_FALSE(testEdge_.jacobianOplusXn<2>().array().isNaN().any())
      << "Jacobian contains NaN";

  ASSERT_FALSE(testEdge_.jacobianOplusXn<0>().array().isInf().any())
      << "Jacobian not finite";
  ASSERT_FALSE(testEdge_.jacobianOplusXn<1>().array().isInf().any())
      << "Jacobian not finite";
  ASSERT_FALSE(testEdge_.jacobianOplusXn<2>().array().isInf().any())
      << "Jacobian not finite";

  ASSERT_LE(0, testEdge_.jacobianOplusXn<0>().array().abs().maxCoeff())
      << "Jacobian is zero";
  ASSERT_LE(0, testEdge_.jacobianOplusXn<1>().array().abs().maxCoeff())
      << "Jacobian is zero";
  ASSERT_LE(0, testEdge_.jacobianOplusXn<2>().array().abs().maxCoeff())
      << "Jacobian is zero";
}

TEST_F(AutoDifferentiation, ComputesNothingForFixed) {
  v2_->setFixed(true);

  testEdge_.linearizeOplus(jacobianWorkspace_);

  ASSERT_LE(0, testEdge_.jacobianOplusXn<0>().array().abs().maxCoeff())
      << "Jacobian is zero";
  ASSERT_DOUBLE_EQ(0, testEdge_.jacobianOplusXn<1>().array().abs().maxCoeff())
      << "Jacobian is non-zero";
  ASSERT_LE(0, testEdge_.jacobianOplusXn<2>().array().abs().maxCoeff())
      << "Jacobian is zero";
}

/**
 * Test fixture for performing the AD tests of EdgeSE2. We setup two vertices
 * and edges to perform the tests. As edges we have the default EdgeSE2 from g2o
 * and the EdgeSE2AD which uses an AD implementation.
 */
class AutoDifferentiationEdgeSE2 : public ::testing::Test {
 protected:
  void SetUp() override {
    v1_->setEstimate(g2o::SE2(0, 0, 0));
    v2_->setEstimate(g2o::SE2(1, 1, 1));

    const g2o::SE2 meas(0.5, 0.5, 0.5);

    testEdge_.setMeasurement(meas);
    testEdge_.setVertex(0, v1_);
    testEdge_.setVertex(1, v2_);
    jacobianWorkspace_.updateSize(&testEdge_);
    jacobianWorkspace_.allocate();

    testEdgeAd_.setMeasurement(meas);
    testEdgeAd_.setVertex(0, v1_);
    testEdgeAd_.setVertex(1, v2_);
    jacobianWorkspaceAd_.updateSize(&testEdgeAd_);
    jacobianWorkspaceAd_.allocate();
  }
  std::shared_ptr<g2o::VertexSE2> v1_ = std::make_shared<g2o::VertexSE2>();
  std::shared_ptr<g2o::VertexSE2> v2_ = std::make_shared<g2o::VertexSE2>();
  g2o::EdgeSE2 testEdge_;
  EdgeSE2AD testEdgeAd_;
  g2o::JacobianWorkspace jacobianWorkspace_;
  g2o::JacobianWorkspace jacobianWorkspaceAd_;
};

TEST_F(AutoDifferentiationEdgeSE2, AdComputesSomething) {
  testEdgeAd_.linearizeOplus(jacobianWorkspaceAd_);

#if 0
  std::cerr << PVAR(testEdgeAd.jacobianOplusXn<0>()) << std::endl;
  std::cerr << PVAR(testEdgeAd.jacobianOplusXn<1>()) << std::endl;
#endif

  ASSERT_FALSE(testEdgeAd_.jacobianOplusXn<0>().array().isNaN().any())
      << "Jacobian contains NaN";
  ASSERT_FALSE(testEdgeAd_.jacobianOplusXn<1>().array().isNaN().any())
      << "Jacobian contains NaN";

  ASSERT_FALSE(testEdgeAd_.jacobianOplusXn<0>().array().isInf().any())
      << "Jacobian not finite";
  ASSERT_FALSE(testEdgeAd_.jacobianOplusXn<1>().array().isInf().any())
      << "Jacobian not finite";

  ASSERT_LE(0, testEdgeAd_.jacobianOplusXn<0>().array().abs().maxCoeff())
      << "Jacobian is zero";
  ASSERT_LE(0, testEdgeAd_.jacobianOplusXn<1>().array().abs().maxCoeff())
      << "Jacobian is zero";
}

TEST_F(AutoDifferentiationEdgeSE2, AdComputesCorrect) {
  // ugly syntax to call the Jacobian into a workspace
  testEdge_
      .g2o::BaseBinaryEdge<3, g2o::SE2, g2o::VertexSE2,
                           g2o::VertexSE2>::linearizeOplus(jacobianWorkspace_);
  // here no ugly call needed because we use the base class one directly, see
  // above
  testEdgeAd_.linearizeOplus(jacobianWorkspaceAd_);

  ASSERT_TRUE(
      testEdgeAd_.jacobianOplusXn<0>().isApprox(testEdge_.jacobianOplusXn<0>()))
      << "Jacobian differs";
  ASSERT_TRUE(
      testEdgeAd_.jacobianOplusXn<1>().isApprox(testEdge_.jacobianOplusXn<1>()))
      << "Jacobian differs";
}
