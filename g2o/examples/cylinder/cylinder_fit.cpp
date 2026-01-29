// g2o - General Graph Optimization
// Copyright (C) 2012 R. Kümmerle
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

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/sampler.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "cyRepresentation.h"
#include "data.h"

using namespace std;

int main(int argc, char** argv) {
  int numPoints;
  int maxIterations;
  bool verbose;
  g2o::CommandArgs arg;
  arg.param("numPoints", numPoints, 100,
            "number of points sampled from the circle");
  arg.param("i", maxIterations, 10, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.parseArgs(argc, argv);

  std::vector<Eigen::Vector3d> landmarks;
  std::vector<Eigen::Matrix4d> v_Twc;
  std::vector<Eigen::Matrix4d> v_noisyTwc;

  createLandmarks(landmarks);
  std::vector<Eigen::Vector3d> noisyLandmarks = addLandmarksNoise(landmarks);
  createCameraPose(v_Twc, v_noisyTwc);
  const size_t pose_num = v_Twc.size();

  // cv::Mat cv_K = (cv::Mat_<double>(3, 3) << 480, 0, 320, 0, 480, 240, 0, 0,
  // 1);
  Eigen::Matrix3d K;
  K << 480, 0, 320, 0, 480, 240, 0, 0, 1;
  // cv::cv2eigen(cv_K, K);
  std::cout << K << std::endl;
  std::vector<Eigen::Vector2i> features_curr;
  // Setup optimizer
  g2o::SparseOptimizer optimizer;

  typedef g2o::BlockSolverX BlockSolverType;
  typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
      LinearSolverType;

  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
  optimizer.setAlgorithm(solver);

  double focal_length = 480.;
  Eigen::Vector2d principal_point(320., 240.);

  g2o::CameraParameters* camera =
      new g2o::CameraParameters(focal_length, principal_point, 0);
  camera->setId(0);
  optimizer.addParameter(camera);

  // Add pose vertex
  Eigen::Vector2d obs;

  CylinderFittingVertex* v = new CylinderFittingVertex();
  Eigen::VectorXd abc(5);
  abc << 0.1, 0.1, 23.0, -2.0, 10.0;
  v->setEstimate(abc);
  v->setId(0);

  optimizer.addVertex(v);

  for (size_t j = 0; j < noisyLandmarks.size(); j++) {
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    //        std::cout << noisyLandmarks[j].transpose() << std::endl;
    vPoint->setEstimate(noisyLandmarks[j]);
    vPoint->setId(j + 3);
    //        vPoint->setFixed(true);
    //        vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
  }

  std::vector<CylinderFittingEdge*> vcy;

  // 往图中增加边
  for (size_t i = 0; i < noisyLandmarks.size(); i++) {
    CylinderFittingEdge* edge2 = new CylinderFittingEdge();
    edge2->setId(noisyLandmarks.size() * 2 + i);
    edge2->setVertex(0, optimizer.vertices()[i + 3]);  // 设置连接的顶点
    edge2->setVertex(1, optimizer.vertices()[0]);
    edge2->setInformation(
        Eigen::Matrix<double, 1, 1>::Identity());  // 信息矩阵：协方差矩阵之逆
    optimizer.addEdge(edge2);
    vcy.push_back(edge2);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(40);
  return 0;
}
