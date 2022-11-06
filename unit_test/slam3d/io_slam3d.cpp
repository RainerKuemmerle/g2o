// g2o - General Graph Optimization
// Copyright (C) 2014 R. Kuemmerle, G. Grisetti, W. Burgard
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

#include <memory>
#include <sstream>

#include "g2o/types/slam3d/edge_pointxyz.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_offset.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_depth.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_disparity.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/edge_se3_xyzprior.h"
#include "g2o/types/slam3d/edge_xyz_prior.h"
#include "g2o/types/slam3d/parameter_stereo_camera.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/io.h"
#include "unit_test/test_helper/random_state.h"

using namespace g2o; //NOLINT

class IoSlam3dParam : public ::testing::Test {
 protected:
  void SetUp() override {
    graph_ = std::make_unique<g2o::OptimizableGraph>();

    // setting up parameters for tests
    paramOffset_.reset(new ParameterSE3Offset());
    paramOffset_->setId(paramId_++);
    graph_->addParameter(paramOffset_);

    paramCamera_.reset(new ParameterCamera);
    paramCamera_->setId(paramId_++);
    graph_->addParameter(paramCamera_);

    // setting up some vertices
    for (int i = 0; i < 2; ++i) {
      auto p = std::make_shared<VertexSE3>();
      p->setId(numVertices_++);
      graph_->addVertex(p);
      poses_.at(i) = p;
    }
    point_.reset(new VertexPointXYZ);
    point_->setId(numVertices_++);
    graph_->addVertex(point_);
  }

  template <typename T>
  bool preparePosePoseEdge(T& e) {
    e->setParameterId(0, paramOffset_->id());
    for (size_t i = 0; i < e->vertices().size(); ++i)
      e->setVertex(i, poses_.at(i));
    return graph_->addEdge(e);
  }

  template <typename T>
  bool preparePosePointEdge(T& e) {
    e->setParameterId(0, paramOffset_->id());
    e->setVertex(0, poses_.at(0));
    e->setVertex(1, point_);
    return graph_->addEdge(e);
  }

  template <typename T>
  bool prepareCamPointEdge(T& e) {
    e->setParameterId(0, paramCamera_->id());
    e->setVertex(0, poses_.at(0));
    e->setVertex(1, point_);
    return graph_->addEdge(e);
  }

  std::unique_ptr<g2o::OptimizableGraph> graph_;
  std::shared_ptr<VertexPointXYZ> point_;
  std::shared_ptr<ParameterSE3Offset> paramOffset_;
  std::shared_ptr<ParameterCamera> paramCamera_;
  std::vector<std::shared_ptr<VertexSE3>> poses_ = {nullptr, nullptr};
  int numVertices_ = 0;
  int paramId_ = 42;
};

/*
 * VERTEX Tests
 */
TEST(IoSlam3d, ReadWriteVertexSE3) {
  readWriteVectorBasedVertex<VertexSE3, internal::RandomIsometry3>();
}

TEST(IoSlam3d, ReadWriteVertexPointXYZ) {
  readWriteVectorBasedVertex<VertexPointXYZ>();
}

/*
 * EDGE Tests
 */
TEST(IoSlam3d, ReadWriteEdgeSE3) {
  readWriteVectorBasedEdge<EdgeSE3, internal::RandomIsometry3>();
}

TEST(IoSlam3d, ReadWriteEdgePointXYZ) {
  readWriteVectorBasedEdge<EdgePointXYZ>();
}

TEST(IoSlam3d, ReadWriteEdgeXYZPrior) {
  readWriteVectorBasedEdge<EdgeXYZPrior>();
}

TEST_F(IoSlam3dParam, ReadWriteEdgeSE3Offset) {
  // additional graph structures
  auto paramOffset2 = std::make_shared<ParameterSE3Offset>();
  paramOffset2->setId(paramId_++);
  graph_->addParameter(paramOffset2);

  // setting up the edge for output
  auto outputEdge = std::make_shared<EdgeSE3Offset>();
  outputEdge->setParameterId(1, paramOffset2->id());
  ASSERT_TRUE(preparePosePoseEdge(outputEdge));

  // Test IO
  readWriteVectorBasedEdge<EdgeSE3Offset, internal::RandomIsometry3>(
      outputEdge);
}

TEST_F(IoSlam3dParam, ReadWriteEdgeSE3PointXYZ) {
  auto outputEdge = std::make_shared<EdgeSE3PointXYZ>();
  ASSERT_TRUE(preparePosePointEdge(outputEdge));
  readWriteVectorBasedEdge<EdgeSE3PointXYZ>(outputEdge);
}

TEST_F(IoSlam3dParam, ReadWriteEdgeSE3Prior) {
  auto outputEdge = std::make_shared<EdgeSE3Prior>();
  ASSERT_TRUE(preparePosePoseEdge(outputEdge));
  readWriteVectorBasedEdge<EdgeSE3Prior, internal::RandomIsometry3>(outputEdge);
}

TEST_F(IoSlam3dParam, ReadWriteEdgeSE3PointXYZDepth) {
  auto outputEdge = std::make_shared<EdgeSE3PointXYZDepth>();
  ASSERT_TRUE(prepareCamPointEdge(outputEdge));
  readWriteVectorBasedEdge<EdgeSE3PointXYZDepth>(outputEdge);
}

TEST_F(IoSlam3dParam, ReadWriteEdgeSE3PointXYZDisparity) {
  auto outputEdge = std::make_shared<EdgeSE3PointXYZDisparity>();
  ASSERT_TRUE(prepareCamPointEdge(outputEdge));
  readWriteVectorBasedEdge<EdgeSE3PointXYZDisparity>(outputEdge);
}

TEST_F(IoSlam3dParam, ReadWriteEdgeSE3XYZPrior) {
  auto outputEdge = std::make_shared<EdgeSE3XYZPrior>();
  ASSERT_TRUE(preparePosePoseEdge(outputEdge));
  readWriteVectorBasedEdge<EdgeSE3XYZPrior>(outputEdge);
}

// Parameter tests
TEST(IoSlam3d, ReadWriteParameterCamera) {
  ParameterCamera outputParam;
  outputParam.setOffset(internal::RandomIsometry3::create());
  outputParam.setKcam(1, 2, 3, 4);  // just some test values for read/write
  ParameterCamera inputParam;
  readWriteGraphElement(outputParam, &inputParam);
  ASSERT_TRUE(internal::RandomIsometry3::isApprox(outputParam.offset(),
                                                  inputParam.offset()));
  ASSERT_TRUE(outputParam.Kcam().isApprox(inputParam.Kcam(), 1e-5));
}

TEST(IoSlam3d, ReadWriteParameterStereoCamera) {
  ParameterStereoCamera outputParam;
  outputParam.setOffset(internal::RandomIsometry3::create());
  outputParam.setBaseline(0.1);
  outputParam.setKcam(1, 2, 3, 4);  // just some test values for read/write
  ParameterStereoCamera inputParam;
  readWriteGraphElement(outputParam, &inputParam);
  ASSERT_DOUBLE_EQ(outputParam.baseline(), inputParam.baseline());
  ASSERT_TRUE(internal::RandomIsometry3::isApprox(outputParam.offset(),
                                                  inputParam.offset()));
  ASSERT_TRUE(outputParam.Kcam().isApprox(inputParam.Kcam(), 1e-5));
}

TEST(IoSlam3d, ReadWriteParameterSE3Offset) {
  ParameterSE3Offset outputParam;
  outputParam.setOffset(internal::RandomIsometry3::create());
  ParameterSE3Offset inputParam;
  readWriteGraphElement(outputParam, &inputParam);
  ASSERT_TRUE(internal::RandomIsometry3::isApprox(outputParam.offset(),
                                                  inputParam.offset()));
}
