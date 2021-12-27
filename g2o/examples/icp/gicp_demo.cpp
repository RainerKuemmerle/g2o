// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
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
#include <random>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/stuff/sampler.h"
#include "g2o/types/icp/types_icp.h"

using std::cerr;
using std::cout;
using std::endl;

namespace g2o {
static int gicp_demo() {
  double euc_noise = 0.01;  // noise in position, m
  //  double outlier_ratio = 0.1;

  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  // variable-size block solver
  std::unique_ptr<g2o::OptimizationAlgorithm> solver(
      new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverX>(
          g2o::make_unique<
              LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>>())));

  optimizer.setAlgorithm(std::move(solver));

  std::vector<Vector3> true_points;
  for (size_t i = 0; i < 1000; ++i) {
    true_points.emplace_back((g2o::Sampler::uniformRand(0., 1.) - 0.5) * 3,
                             g2o::Sampler::uniformRand(0., 1.) - 0.5,
                             g2o::Sampler::uniformRand(0., 1.) + 10);
  }

  // set up two poses
  int vertex_id = 0;
  for (size_t i = 0; i < 2; ++i) {
    // set up rotation and translation for this node
    Vector3 t(0, 0, i);
    Quaternion q;
    q.setIdentity();

    Eigen::Isometry3d cam;  // camera pose
    cam = q;
    cam.translation() = t;

    // set up node
    auto vc = std::make_shared<VertexSE3>();
    vc->setEstimate(cam);

    vc->setId(vertex_id);  // vertex id

    cerr << t.transpose() << " | " << q.coeffs().transpose() << endl;

    // set first cam pose fixed
    if (i == 0) vc->setFixed(true);

    // add to optimizer
    optimizer.addVertex(vc);

    vertex_id++;
  }

  // set up point matches
  for (size_t i = 0; i < true_points.size(); ++i) {
    // get two poses
    auto vp0 = std::dynamic_pointer_cast<VertexSE3>(
        optimizer.vertices().find(0)->second);
    auto vp1 = std::dynamic_pointer_cast<VertexSE3>(
        optimizer.vertices().find(1)->second);

    // calculate the relative 3D position of the point
    Vector3 pt0 = vp0->estimate().inverse() * true_points[i];
    Vector3 pt1 = vp1->estimate().inverse() * true_points[i];

    // add in noise
    pt0 += Vector3(g2o::Sampler::gaussRand(0., euc_noise),
                   g2o::Sampler::gaussRand(0., euc_noise),
                   g2o::Sampler::gaussRand(0., euc_noise));

    pt1 += Vector3(g2o::Sampler::gaussRand(0., euc_noise),
                   g2o::Sampler::gaussRand(0., euc_noise),
                   g2o::Sampler::gaussRand(0., euc_noise));

    // form edge, with normals in varioius positions
    Vector3 nm0(0, i, 1);
    Vector3 nm1(0, i, 1);
    nm0.normalize();
    nm1.normalize();

    auto e  // new edge with correct cohort for caching
        = std::make_shared<EdgeVVGicp>();

    e->setVertex(0, vp0);  // first viewpoint
    e->setVertex(1, vp1);  // second viewpoint

    EdgeGICP meas;
    meas.pos0 = pt0;
    meas.pos1 = pt1;
    meas.normal0 = nm0;
    meas.normal1 = nm1;

    e->setMeasurement(meas);
    //        e->inverseMeasurement().pos() = -kp;

    meas = e->measurement();
    // use this for point-plane
    e->information() = meas.prec0(0.01);

    // use this for point-point
    //    e->information().setIdentity();

    //    e->setRobustKernel(true);
    // e->setHuberWidth(0.01);

    optimizer.addEdge(e);
  }

  // move second cam off of its true position
  auto vc = std::dynamic_pointer_cast<VertexSE3>(
      optimizer.vertices().find(1)->second);
  Eigen::Isometry3d cam = vc->estimate();
  cam.translation() = Vector3(0, 0, 0.2);
  vc->setEstimate(cam);

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << endl;

  optimizer.setVerbose(true);

  optimizer.optimize(5);

  cout << endl << "Second vertex should be near 0,0,1" << endl;
  cout << std::dynamic_pointer_cast<VertexSE3>(
              optimizer.vertices().find(0)->second)
              ->estimate()
              .translation()
              .transpose()
       << endl;
  cout << std::dynamic_pointer_cast<VertexSE3>(
              optimizer.vertices().find(1)->second)
              ->estimate()
              .translation()
              .transpose()
       << endl;
  return 0;
}
}  // namespace g2o

int main() { return g2o::gicp_demo(); }
