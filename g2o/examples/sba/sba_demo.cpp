// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
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

#include <cstdint>
#include <iostream>
#include <unordered_set>

#include "g2o/config.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/icp/types_icp.h"

#if defined G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#else
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#endif

using std::cerr;
using std::cout;
using std::endl;

class Sample {
 public:
  static int uniform(int from, int to) {
    return static_cast<int>(g2o::Sampler::uniformRand(from, to));
  }
};

int main(int argc, const char* argv[]) {
  if (argc < 2) {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] "
            "[STRUCTURE_ONLY] [DENSE]"
         << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)"
         << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)"
         << endl;
    cout << "STRUCTURE_ONLY: performed structure-only BA to get better point "
            "initializations (0 or "
            "1; default: 0==false)"
         << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to "
            "1==true."
         << endl;
    cout << endl;
    exit(0);
  }

  const double PIXEL_NOISE = atof(argv[1]);

  double OUTLIER_RATIO = 0.0;

  if (argc > 2) {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc > 3) {
    ROBUST_KERNEL = atoi(argv[3]) != 0;
  }
  bool STRUCTURE_ONLY = false;
  if (argc > 4) {
    STRUCTURE_ONLY = atoi(argv[4]) != 0;
  }

  bool DENSE = false;
  if (argc > 5) {
    DENSE = atoi(argv[5]) != 0;
  }

  cout << "PIXEL_NOISE: " << PIXEL_NOISE << endl;
  cout << "OUTLIER_RATIO: " << OUTLIER_RATIO << endl;
  cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
  cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY << endl;
  cout << "DENSE: " << DENSE << endl;

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
  if (DENSE) {
    linearSolver = g2o::make_unique<
        g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    cerr << "Using DENSE" << endl;
  } else {
#ifdef G2O_HAVE_CHOLMOD
    cerr << "Using CHOLMOD" << endl;
    linearSolver = g2o::make_unique<
        g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
#else
    linearSolver = g2o::make_unique<
        g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    cerr << "Using CSPARSE" << endl;
#endif
  }

  std::unique_ptr<g2o::OptimizationAlgorithm> solver(
      new g2o::OptimizationAlgorithmLevenberg(
          g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))));

  optimizer.setAlgorithm(std::move(solver));

  // set up 500 points
  std::vector<g2o::Vector3> true_points;
  for (size_t i = 0; i < 500; ++i) {
    true_points.emplace_back((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
                             g2o::Sampler::uniformRand(0., 1.) - 0.5,
                             g2o::Sampler::uniformRand(0., 1.) + 10);
  }

  const g2o::Vector2 focal_length(500, 500);     // pixels
  const g2o::Vector2 principal_point(320, 240);  // 640x480 image
  constexpr double kBaseline = 0.075;            // 7.5 cm baseline

  std::vector<g2o::Isometry3, Eigen::aligned_allocator<g2o::Isometry3>>
      true_poses;

  // set up camera params
  g2o::VertexSCam::setKcam(focal_length[0], focal_length[1], principal_point[0],
                           principal_point[1], kBaseline);

  // set up 5 vertices, first 2 fixed
  int vertex_id = 0;
  for (size_t i = 0; i < 5; ++i) {
    const g2o::Vector3 trans(i * 0.04 - 1., 0, 0);

    Eigen::Quaterniond q;
    q.setIdentity();
    g2o::Isometry3 pose;
    pose = q;
    pose.translation() = trans;

    auto v_se3 = std::make_shared<g2o::VertexSCam>();

    v_se3->setId(vertex_id);
    v_se3->setEstimate(pose);
    v_se3->setAll();  // set aux transforms

    if (i < 2) v_se3->setFixed(true);

    optimizer.addVertex(v_se3);
    true_poses.push_back(pose);
    vertex_id++;
  }

  int point_id = vertex_id;
  double sum_diff2 = 0;

  cout << endl;
  std::unordered_map<int, int> pointid_2_trueid;
  std::unordered_set<int> inliers;

  // add point projections to this vertex
  for (size_t i = 0; i < true_points.size(); ++i) {
    auto v_p = std::make_shared<g2o::VertexPointXYZ>();

    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->setEstimate(true_points.at(i) +
                     g2o::Vector3(g2o::Sampler::gaussRand(0., 1),
                                  g2o::Sampler::gaussRand(0., 1),
                                  g2o::Sampler::gaussRand(0., 1)));

    int num_obs = 0;

    for (size_t j = 0; j < true_poses.size(); ++j) {
      g2o::Vector3 z;
      dynamic_cast<g2o::VertexSCam*>(optimizer.vertices().find(j)->second.get())
          ->mapPoint(z, true_points.at(i));

      if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
        ++num_obs;
      }
    }

    if (num_obs >= 2) {
      optimizer.addVertex(v_p);

      bool inlier = true;
      for (size_t j = 0; j < true_poses.size(); ++j) {
        g2o::Vector3 z;
        dynamic_cast<g2o::VertexSCam*>(
            optimizer.vertices().find(j)->second.get())
            ->mapPoint(z, true_points.at(i));

        if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
          const double sam = g2o::Sampler::uniformRand(0., 1.);
          if (sam < OUTLIER_RATIO) {
            z = g2o::Vector3(Sample::uniform(64, 640), Sample::uniform(0, 480),
                             Sample::uniform(0, 64));  // disparity
            z(2) = z(0) - z(2);                        // px' now

            inlier = false;
          }

          z += g2o::Vector3(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                            g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                            g2o::Sampler::gaussRand(0., PIXEL_NOISE / 16.0));

          auto e = std::make_shared<g2o::EdgeXyzVsc>();

          e->vertices()[0] = v_p;

          e->vertices()[1] = optimizer.vertices().find(j)->second;

          e->setMeasurement(z);
          // e->inverseMeasurement() = -z;
          e->information() = g2o::Matrix3::Identity();

          if (ROBUST_KERNEL) {
            e->setRobustKernel(std::make_shared<g2o::RobustKernelHuber>());
          }

          optimizer.addEdge(e);
        }
      }

      if (inlier) {
        inliers.insert(point_id);
        const g2o::Vector3 diff = v_p->estimate() - true_points[i];

        sum_diff2 += diff.dot(diff);
      }
      // else
      //   cout << "Point: " << point_id <<  "has at least one spurious
      //   observation" <<endl;

      pointid_2_trueid.insert(std::make_pair(point_id, i));

      ++point_id;
    }
  }

  cout << endl;
  optimizer.initializeOptimization();

  optimizer.setVerbose(true);

  if (STRUCTURE_ONLY) {
    cout << "Performing structure-only BA:" << endl;
    g2o::StructureOnlySolver<3> structure_only_ba;
    g2o::OptimizableGraph::VertexContainer points;
    for (const auto& it : optimizer.vertices()) {
      auto v =
          std::static_pointer_cast<g2o::OptimizableGraph::Vertex>(it.second);
      if (v->dimension() == 3) points.push_back(v);
    }

    structure_only_ba.calc(points, 10);
  }

  cout << endl;
  cout << "Performing full BA:" << endl;
  optimizer.optimize(10);

  cout << endl;
  cout << "Point error before optimisation (inliers only): "
       << sqrt(sum_diff2 / inliers.size()) << endl;

  sum_diff2 = 0;

  for (auto& it : pointid_2_trueid) {
    auto v_it = optimizer.vertices().find(it.first);

    if (v_it == optimizer.vertices().end()) {
      cerr << "Vertex " << it.first << " not in graph!" << endl;
      exit(-1);
    }

    auto v_p = std::dynamic_pointer_cast<g2o::VertexPointXYZ>(v_it->second);

    if (v_p == nullptr) {
      cerr << "Vertex " << it.first << "is not a PointXYZ!" << endl;
      exit(-1);
    }

    const g2o::Vector3 diff = v_p->estimate() - true_points[it.second];

    if (inliers.find(it.first) == inliers.end()) continue;

    sum_diff2 += diff.dot(diff);
  }

  cout << "Point error after optimisation (inliers only): "
       << sqrt(sum_diff2 / inliers.size()) << endl;
  cout << endl;
}
