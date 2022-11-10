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
#include <iostream>

#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "simulator.h"
#include "types_tutorial_slam2d.h"
#include "vertex_point_xy.h"
#include "vertex_se2.h"

using std::cerr;
using std::endl;

namespace g2o {
namespace tutorial {

static int run_slam2d_tutorial() {
  // TODO(Rainer): simulate different sensor offset
  // simulate a robot observing landmarks while traveling on a grid
  const SE2 sensorOffsetTransf(0.2, 0.1, -0.1);
  const int numNodes = 300;
  Simulator simulator;
  simulator.simulate(numNodes, sensorOffsetTransf);

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  using SlamBlockSolver = BlockSolver<BlockSolverTraits<-1, -1>>;
  using SlamLinearSolver = LinearSolverEigen<SlamBlockSolver::PoseMatrixType>;

  // allocating the optimizer
  SparseOptimizer optimizer;
  auto linearSolver = g2o::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  std::unique_ptr<OptimizationAlgorithm> solver(
      new OptimizationAlgorithmGaussNewton(
          g2o::make_unique<SlamBlockSolver>(std::move(linearSolver))));

  optimizer.setAlgorithm(std::move(solver));

  // add the parameter representing the sensor offset
  auto sensorOffset = std::make_shared<ParameterSE2Offset>();
  sensorOffset->setOffset(sensorOffsetTransf);
  sensorOffset->setId(0);
  optimizer.addParameter(sensorOffset);

  // adding the odometry to the optimizer
  // first adding all the vertices
  cerr << "Optimization: Adding robot poses ... ";
  for (const auto& p : simulator.poses()) {
    const SE2& t = p.simulatorPose;
    auto robot = std::make_shared<VertexSE2>();
    robot->setId(p.id);
    robot->setEstimate(t);
    optimizer.addVertex(robot);
  }
  cerr << "done." << endl;

  // second add the odometry constraints
  cerr << "Optimization: Adding odometry measurements ... ";
  for (const auto& simEdge : simulator.odometry()) {
    auto odometry = std::make_shared<EdgeSE2>();
    odometry->vertices()[0] = optimizer.vertex(simEdge.from);
    odometry->vertices()[1] = optimizer.vertex(simEdge.to);
    odometry->setMeasurement(simEdge.simulatorTransf);
    odometry->setInformation(simEdge.information);
    optimizer.addEdge(odometry);
  }
  cerr << "done." << endl;

  // add the landmark observations
  cerr << "Optimization: add landmark vertices ... ";
  for (const auto& l : simulator.landmarks()) {
    auto landmark = std::make_shared<VertexPointXY>();
    landmark->setId(l.id);
    landmark->setEstimate(l.simulatedPose);
    optimizer.addVertex(landmark);
  }
  cerr << "done." << endl;

  cerr << "Optimization: add landmark observations ... ";
  for (const auto& simEdge : simulator.landmarkObservations()) {
    auto landmarkObservation = std::make_shared<EdgeSE2PointXY>();
    landmarkObservation->vertices()[0] = optimizer.vertex(simEdge.from);
    landmarkObservation->vertices()[1] = optimizer.vertex(simEdge.to);
    landmarkObservation->setMeasurement(simEdge.simulatorMeas);
    landmarkObservation->setInformation(simEdge.information);
    landmarkObservation->setParameterId(0, sensorOffset->id());
    optimizer.addEdge(landmarkObservation);
  }
  cerr << "done." << endl;

  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("tutorial_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  auto firstRobotPose =
      std::dynamic_pointer_cast<VertexSE2>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  cerr << "Optimizing" << endl;
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  cerr << "done." << endl;

  optimizer.save("tutorial_after.g2o");

  // freeing the graph memory
  optimizer.clear();

  return 0;
}

}  // namespace tutorial
}  // namespace g2o

int main() { return g2o::tutorial::run_slam2d_tutorial(); }
